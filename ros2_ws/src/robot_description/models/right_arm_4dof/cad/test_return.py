#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
4.4 四自由度机械臂：MuJoCo 重力补偿 + 有上限的回弹阻抗 + 中文上位机。

特点：
  1. 在上位机设定 J1~J4 的目标角度；人手掰开后，松手会尝试回到目标角度。
  2. 当角度误差较大时，位置恢复力矩不继续随角度增大：
         tau_spring = clip(K * (q_goal - q), -tau_limit, +tau_limit)
     目标附近保留线性小力矩区，降低目标附近的冲击/抖动。
  3. tau_tff = gravity_scale * g(q) + tau_spring - D * dq
     MuJoCo 计算完整四关节重力力矩，四个关节不是独立的正弦近似。
  4. MIT 的 P、V、Kp、Kd 都发零，所有实际控制力矩通过 tff 发出。
     这里的 K 和 D 是上位机外环增益，不是 MIT 中的 Kp/Kd 原始码。
  5. SocketCAN can0、CAN-FD BRS、关节ID依次1、2、3、4。

警告：限制的是指令中的“恢复力矩项”，不是人手实际感受到的恒定力。
摩擦、传动效率、阻尼、重力误差、反馈时延均会影响真实力感和回位精度。
零tff的MIT使能也可能释放抱闸。首次测试必须有机械支撑和独立急停。

依赖：pip install mujoco python-can pillow
WSL中文字体：sudo apt install fonts-noto-cjk
自测：python3 impedance_return_4joint_can0_cn.py --self-test
模型：python3 impedance_return_4joint_can0_cn.py --check-model
演示：python3 impedance_return_4joint_can0_cn.py --demo
实机：python3 impedance_return_4joint_can0_cn.py
"""

import argparse
import math
import os
import subprocess
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from collections import deque
from tkinter import ttk

# ======================= 1. 配置 =======================
CAN_CHANNEL = "can0"
MODEL_PATH = "./urdf/4.4.mjcf.xml"
NODE_IDS = [1, 2, 3, 4]
JOINT_NAMES = [f"Joint_link{i}" for i in range(1, 5)]
JOINT_LIMITS = [(-90, 90), (-180, 180), (-135, 135), (-180, 180)]

# 已写入电机的 MIT tff 物理量程：J1=58lite，J2~4=48lite
T_MAX = [50.0, 12.0, 12.0, 12.0]

# 电机输出侧角度归零与正方向，需要同当前 MJCF 的角度定义匹配。
ZERO_DEG = [0.0, 0.0, 0.0, 0.0]
ANGLE_SIGN = [1, 1, 1, 1]
TORQUE_SIGN = [1, 1, 1, 1]

CONTROL_DT = 0.005       # 5 ms，目标200 Hz（Python非硬实时）
FEEDBACK_TIMEOUT = 0.20 # 必需的简易超时，避免一直使用过时角度
FF_ABS_CAP = [5.0, 2.0, 2.0, 2.0]   # 合成tff的额外软件幅值上限，不是硬件限矩
FF_STEP = [0.06, 0.03, 0.03, 0.03]  # 每5ms命令最大变化Nm

# 每个关节的目标角度(度)、外环刚度(Nm/rad)、恢复力矩上限(Nm)、阻尼(Nm.s/rad)、重力比例。
# 重力比例默认0以方便先核对力矩方向；之后应逐步增加。
DEFAULT_SETTINGS = [
    dict(goal=0.0, k=10.0, cap=1.0, d=0.25, gravity=0.0),
    dict(goal=0.0, k=7.0,  cap=0.5, d=0.15, gravity=0.0),
    dict(goal=0.0, k=7.0,  cap=0.5, d=0.15, gravity=0.0),
    dict(goal=0.0, k=5.0,  cap=0.3, d=0.10, gravity=0.0),
]

# ======================= 2. 线程共享状态 =======================
lock = threading.Lock()
stop_event = threading.Event()
worker = None
settings = [p.copy() for p in DEFAULT_SETTINGS]
feedback = [None] * 4          # (接收时间, 反馈角度deg, 电机rpm, Iq_mA, fault, status)
feedback_speed = [0.0] * 4     # 由反馈角度差分得到的输出轴速度 deg/s
rx_times = [deque(maxlen=250) for _ in range(4)]
rx_count = [0] * 4             # 独立整数计数器，不与GUI字符串混用
history = deque(maxlen=4000)  # (timestamp, 真实模型角度[4], 目标模型角度[4])
state = {
    "status": "未启动", "error": "", "tx_count": 0,
    "rows": [{} for _ in range(4)], "goal_revision": 0,
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def encode(value, lo, hi, bits):
    x = clamp(value, lo, hi)
    return int(math.floor((x-lo) * ((1 << bits)-1)/(hi-lo) + 0.5 + 1e-9))


def decode(value, lo, hi, bits):
    return lo + value * (hi-lo)/((1 << bits)-1)


def mit_payload(tff_nm, tmax, enable=True):
    """纯力矩 MIT：位置/速度字段编码为零，Kp/Kd原始码=0。"""
    p_raw = encode(0.0, -3.14, 3.14, 16)
    v_raw = encode(0.0, -3.14, 3.14, 12)
    kp_raw = kd_raw = 0
    t_raw = encode(tff_nm, -tmax, tmax, 12)
    b = bytes([
        0xCC if enable else 0x0C,
        p_raw >> 8, p_raw & 255,
        v_raw >> 4, ((v_raw & 15) << 4) | (kp_raw >> 8), kp_raw & 255,
        kd_raw >> 4, ((kd_raw & 15) << 4) | (t_raw >> 8), t_raw & 255,
        0, 0, 0,
    ])
    return b, t_raw, decode(t_raw, -tmax, tmax, 12)


def send_mit(bus, can, index, torque, enable=True):
    payload, t_raw, physical_sent = mit_payload(torque, T_MAX[index], enable)
    frame = can.Message(
        arbitration_id=0x110 | NODE_IDS[index], is_extended_id=False,
        is_fd=True, bitrate_switch=True, data=payload, check=True)
    bus.send(frame, timeout=0.04)
    return t_raw, physical_sent


def parse_feedback(can_id, buf, timestamp=None):
    """电机自动回复：兼容标准反馈ID 0x301~304 和 0x001~004。"""
    if len(buf) != 12:
        return None
    for index, node in enumerate(NODE_IDS):
        if can_id in (0x300 | node, node):
            stamp = time.monotonic() if timestamp is None else timestamp
            angle = int.from_bytes(buf[0:2], "big", signed=True)*180.0/32768.0
            rpm = int.from_bytes(buf[2:4], "big", signed=True)
            iq = int.from_bytes(buf[4:6], "big", signed=True)
            fault = int.from_bytes(buf[6:8], "big", signed=False)
            return index, (stamp, angle, rpm, iq, fault, buf[11])
    return None


def receive_loop(bus):
    """只接收每次MIT后自动返回的状态帧。"""
    try:
        while not stop_event.is_set():
            msg = bus.recv(timeout=0.03)
            if msg is None or msg.is_extended_id or msg.is_error_frame:
                continue
            decoded = parse_feedback(msg.arbitration_id, bytes(msg.data))
            if decoded is None:
                continue
            i, fb = decoded
            with lock:
                previous = feedback[i]
                if previous is not None:
                    dt = fb[0] - previous[0]
                    if 0.0005 <= dt <= 0.15:
                        # 角度单圈回绕时使用最短角度差计算瞬时速度
                        difference = (fb[1] - previous[1] + 180.0) % 360.0 - 180.0
                        raw_speed = difference/dt
                        feedback_speed[i] = 0.78*feedback_speed[i] + 0.22*raw_speed
                    else:
                        feedback_speed[i] = 0.0
                feedback[i] = fb
                rx_times[i].append(fb[0])
                rx_count[i] += 1
    except Exception as exc:
        with lock:
            state["error"] = f"CAN接收异常：{exc}"


def locate_model():
    base = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        MODEL_PATH, "./4.4.mjcf.xml", "./urdf/4.4.xml",
        os.path.join(base, "urdf/4.4.mjcf.xml"),
        os.path.join(base, "4.4.mjcf.xml"),
    ]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    raise FileNotFoundError("找不到 4.4.mjcf.xml；请修改脚本顶部 MODEL_PATH")


def load_model():
    """优先读取原MJCF；缺少STL时去掉可视化网格，但保留质量、质心和关节。"""
    import mujoco
    path = locate_model()
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as exc:
        msg = str(exc).lower()
        if not any(w in msg for w in ("mesh", "stl", "resource", "asset", "no such file")):
            raise
        root = ET.parse(path).getroot()
        for parent in root.iter():
            for child in list(parent):
                if child.tag == "geom" or child.tag == "asset":
                    parent.remove(child)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        print("找不到部分STL，已仅加载关节和惯性参数计算重力。")
    if model.nq != 4 or model.nv != 4:
        raise RuntimeError(f"MJCF应含4个活动关节，实际nq={model.nq} nv={model.nv}")
    for i, name in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0 or model.jnt_qposadr[jid] != i or model.jnt_dofadr[jid] != i:
            raise RuntimeError(f"MJCF关节名称或顺序不匹配：{name}")
    return mujoco, model, mujoco.MjData(model)


def gravity_torques(mujoco, model, data, q_degrees):
    """qvel=0 时，qfrc_bias 为该四关节姿态下的联合重力补偿力矩。"""
    data.qpos[:] = [math.radians(q) for q in q_degrees]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return [float(g) for g in data.qfrc_bias]


def restoring_torque(q_now_deg, q_goal_deg, stiffness, max_torque):
    """目标远时固定幅值，目标附近线性渐弱，避免目标处方向抖动。"""
    error_rad = math.radians(q_goal_deg - q_now_deg)
    return clamp(stiffness*error_rad, -max_torque, max_torque)


def compute_four_joint(q_deg, dq_deg_s, goal_params, gravity_nm):
    """纯控制数学：返回4个关节的各项力矩，不读写CAN。"""
    result = []
    for i in range(4):
        p = goal_params[i]
        restore = restoring_torque(q_deg[i], p["goal"], p["k"], p["cap"])
        damp = -p["d"]*math.radians(dq_deg_s[i])
        g_scaled = p["gravity"]*gravity_nm[i]
        torque = TORQUE_SIGN[i]*(g_scaled+restore+damp)
        limited = clamp(torque, -FF_ABS_CAP[i], FF_ABS_CAP[i])
        result.append(dict(
            angle=q_deg[i], goal=p["goal"], error=p["goal"]-q_deg[i],
            speed=dq_deg_s[i], gravity=gravity_nm[i], gravity_sent=g_scaled,
            restore=restore, damping=damp, target_tff=torque, capped_tff=limited,
            limited=abs(torque-limited)>1e-8,
        ))
    return result


def get_rx_hz(timestamps):
    if len(timestamps)<2:
        return 0.0
    duration = timestamps[-1]-timestamps[0]
    return (len(timestamps)-1)/duration if duration>0 else 0.0


def run_real(capture_at_start):
    """简化控制线程：建立反馈 -> 重力/回弹/阻尼计算 -> 每5ms发送四轴tff。"""
    bus = None
    rx_thread = None
    last_ff = [0.0]*4
    started = False
    tx = 0
    try:
        import can
        mj, model, data = load_model()
        bus = can.Bus(interface="socketcan", channel=CAN_CHANNEL,
                      fd=True, receive_own_messages=False)
        rx_thread = threading.Thread(target=receive_loop, args=(bus,), daemon=True)
        rx_thread.start()
        start_time = time.monotonic()
        next_tick = start_time
        with lock:
            state["status"] = "零力矩MIT启动，等待四轴回帧"
        while not stop_event.is_set():
            now = time.monotonic()
            with lock:
                fbs = feedback.copy()
                speed_raw = feedback_speed.copy()
                current_settings = [p.copy() for p in settings]
                timestamps = [list(s) for s in rx_times]
                error = state["error"]
            if error:
                raise RuntimeError(error)
            ready = all(fb is not None for fb in fbs)
            if not ready:
                if now-start_time>3.0:
                    missing = [str(NODE_IDS[i]) for i,v in enumerate(fbs) if v is None]
                    raise TimeoutError("等待反馈超时，ID="+",".join(missing))
            elif any(now-fb[0]>FEEDBACK_TIMEOUT for fb in fbs):
                old = [str(NODE_IDS[i]) for i,fb in enumerate(fbs) if now-fb[0]>FEEDBACK_TIMEOUT]
                raise TimeoutError("电机反馈超时，ID="+",".join(old))

            if ready:
                q_model = [ANGLE_SIGN[i]*(fbs[i][1]-ZERO_DEG[i]) for i in range(4)]
                dq_model = [ANGLE_SIGN[i]*speed_raw[i] for i in range(4)]
                if not started:
                    started = True
                    if capture_at_start:
                        with lock:
                            for i in range(4):
                                settings[i]["goal"] = q_model[i]
                                current_settings[i]["goal"] = q_model[i]
                            state["goal_revision"] += 1
                    with lock:
                        state["status"] = "回弹阻抗运行中"
                g = gravity_torques(mj, model, data, q_model)
                calc = compute_four_joint(q_model, dq_model, current_settings, g)
            else:
                calc = [{} for _ in range(4)]
                q_model = [0.0]*4

            for i in range(4):
                target_ff = calc[i]["capped_tff"] if ready else 0.0
                allowed = FF_STEP[i]
                last_ff[i] += clamp(target_ff-last_ff[i], -allowed, allowed)
                raw, sent = send_mit(bus, can, i, last_ff[i])
                tx += 1
                if ready:
                    calc[i].update(dict(
                        sent=sent, t_raw=raw, iq=fbs[i][3], rpm=fbs[i][2],
                        rx_hz=get_rx_hz(timestamps[i]),
                        rx_age_ms=max(0.0,(now-fbs[i][0])*1000),
                    ))

            with lock:
                state["rows"] = calc
                state["tx_count"] = tx
                if ready:
                    history.append((now,q_model.copy(),[p["goal"] for p in current_settings]))
                    state["status"] = "回弹阻抗运行中"
                else:
                    state["status"] = "正在发送零力矩MIT并等待反馈"

            next_tick += CONTROL_DT
            remaining = next_tick-time.monotonic()
            if remaining < -CONTROL_DT:
                next_tick = time.monotonic()
            else:
                stop_event.wait(max(0.0,remaining))
    except Exception as exc:
        print("控制停止原因：",exc,flush=True)
        with lock:
            state["status"] = "已停止："+str(exc)
    finally:
        stop_event.set()
        if bus is not None:
            for i in range(4):
                try:
                    send_mit(bus, can, i, 0.0, enable=False)
                except Exception:
                    pass
            if rx_thread is not None:
                rx_thread.join(timeout=0.2)
            bus.shutdown()
        with lock:
            if not state["status"].startswith("已停止："):
                state["status"] = "已停止，已请求四轴失能"


def run_demo():
    """无硬件演示：优先用MuJoCo重力计算；未安装MuJoCo时使用示意力矩。"""
    mj = model = data = None
    try:
        mj, model, data = load_model()
    except Exception as exc:
        print("演示未加载MuJoCo，使用示例数值：",str(exc))
    start = time.monotonic()
    while not stop_event.is_set():
        now=time.monotonic()
        t=now-start
        q=[25*math.sin(0.45*t),-15*math.sin(0.40*t),20*math.sin(0.32*t),12*math.cos(0.36*t)]
        dq=[25*0.45*math.cos(0.45*t),-15*0.40*math.cos(0.40*t),20*0.32*math.cos(0.32*t),-12*0.36*math.sin(0.36*t)]
        with lock:
            ps = [p.copy() for p in settings]
        if mj is None:
            g=[-1.15*math.sin(math.radians(q[0])),0.4*math.sin(math.radians(q[1])),
               0.25*math.sin(math.radians(q[2])),0.08*math.sin(math.radians(q[3]))]
        else:
            g=gravity_torques(mj,model,data,q)
        rows=compute_four_joint(q,dq,ps,g)
        for i,row in enumerate(rows):
            raw=encode(row["capped_tff"],-T_MAX[i],T_MAX[i],12)
            row.update(sent=decode(raw,-T_MAX[i],T_MAX[i],12),t_raw=raw,iq=0,rpm=0,rx_hz=0,rx_age_ms=0)
        with lock:
            state["status"]="离线界面演示（非实机测量）"
            state["rows"]=rows
            history.append((now,q,[p["goal"] for p in ps]))
        stop_event.wait(0.04)


def get_chinese_font():
    """部分WSL的Tk会把中文显示为\\uXXXX；文字通过Pillow绘制。"""
    try:
        result = subprocess.check_output(
            ["fc-match","-f","%{file}\n","Noto Sans CJK SC:lang=zh-cn"],
            text=True,timeout=3).strip().splitlines()
        if result and os.path.isfile(result[0]):
            return result[0]
    except (OSError,subprocess.SubprocessError):
        pass
    for p in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "C:/Windows/Fonts/msyh.ttc"):
        if os.path.isfile(p):
            return p
    raise RuntimeError("缺少中文字体：sudo apt install fonts-noto-cjk")


def main(demo=False):
    global worker
    from PIL import Image, ImageDraw, ImageFont, ImageTk

    BG="#F1F5FA"; WHITE="#FFFFFF"; TEXT="#26374A"; BLUE="#1159B6"; GREY="#68788C"
    root=tk.Tk()
    root.title("四关节机械臂 · 恒定回弹力矩阻抗控制")
    root.geometry("1430x700")
    root.minsize(1080,630)
    root.configure(bg=BG)
    font_path=get_chinese_font()
    pictures={}; fonts={}

    def image_label(text, size=14, color=TEXT):
        key=(str(text),size,color)
        if key not in pictures:
            if size not in fonts:
                fonts[size]=ImageFont.truetype(font_path,size)
            f=fonts[size]
            bounds=f.getbbox(str(text))
            pic=Image.new("RGBA", (max(1,bounds[2]-bounds[0]+10),max(1,bounds[3]-bounds[1]+10)))
            ImageDraw.Draw(pic).text((5-bounds[0],5-bounds[1]),str(text),font=f,fill=color)
            pictures[key]=ImageTk.PhotoImage(pic,master=root)
        return pictures[key]

    def zh(parent,text,size=14,bg=WHITE,color=TEXT):
        return tk.Label(parent,image=image_label(text,size,color),bg=bg,bd=0)

    top=tk.Frame(root,bg=BG,padx=20,pady=14); top.pack(fill="x")
    zh(top,"四关节重力补偿 · 限幅回弹阻抗控制",23,bg=BG).pack(anchor="w")
    zh(top,"远离目标后回弹力矩不再增加；松手后尝试回到目标。SocketCAN can0 / 200 Hz",13,bg=BG,color=GREY).pack(anchor="w",pady=(6,0))

    # 可滚动的参数与数据区域，窗口较小时不截断右侧内容。
    body=tk.Frame(root,bg=BG); body.pack(fill="both",expand=True)
    scroll=tk.Scrollbar(body,orient="vertical"); scroll.pack(side="right",fill="y")
    canvas=tk.Canvas(body,bg=BG,highlightthickness=0,yscrollcommand=scroll.set)
    canvas.pack(side="left",fill="both",expand=True)
    scroll.config(command=canvas.yview)
    inner=tk.Frame(canvas,bg=BG,padx=20)
    item=canvas.create_window((0,0),window=inner,anchor="nw")
    inner.bind("<Configure>",lambda e:canvas.config(scrollregion=canvas.bbox("all")))
    canvas.bind("<Configure>",lambda e:canvas.itemconfigure(item,width=e.width))
    canvas.bind_all("<MouseWheel>",lambda e:canvas.yview_scroll(-int(e.delta/120) or (-1 if e.delta>0 else 1),"units"))
    canvas.bind_all("<Button-4>",lambda e:canvas.yview_scroll(-3,"units"))
    canvas.bind_all("<Button-5>",lambda e:canvas.yview_scroll(3,"units"))

    configbox=tk.Frame(inner,bg=WHITE,padx=15,pady=10)
    configbox.pack(fill="x",pady=(0,10))
    zh(configbox,"设置目标角度、恢复刚度及回弹力矩上限",17,color=BLUE).pack(anchor="w",pady=(0,10))
    configgrid=tk.Frame(configbox,bg=WHITE); configgrid.pack(fill="x")
    headings=["关节","目标角度 (°)","刚度 K (Nm/rad)","回弹上限 (Nm)","阻尼 D (Nm·s/rad)","重力比例"]
    for col,label in enumerate(headings):
        zh(configgrid,label,13).grid(row=0,column=col,padx=10,pady=(0,7))
        configgrid.columnconfigure(col,weight=1)
    edits=[]
    for i,p in enumerate(settings):
        zh(configgrid,f"J{i+1} / {'58lite' if i==0 else '48lite'}",13).grid(row=i+1,column=0,padx=5,pady=4)
        current={}
        for col,key in enumerate(("goal","k","cap","d","gravity"),1):
            entry=ttk.Entry(configgrid,width=15,justify="center")
            entry.insert(0,f"{p[key]:g}")
            entry.grid(row=i+1,column=col,sticky="ew",padx=9,pady=3)
            current[key]=entry
        edits.append(current)

    control=tk.Frame(configbox,bg=WHITE)
    control.pack(fill="x",pady=(11,0))

    def apply_settings():
        try:
            new=[]
            for i,inputs in enumerate(edits):
                cfg={key:float(widget.get().strip()) for key,widget in inputs.items()}
                if not all(math.isfinite(x) for x in cfg.values()):
                    raise ValueError(f"J{i+1} 参数不能是NaN或无穷大")
                lo,hi=JOINT_LIMITS[i]
                if not lo <= cfg["goal"] <= hi:
                    raise ValueError(f"J{i+1}目标角度必须在{lo}~{hi}度")
                if not 0 <= cfg["k"] <= 50 or not 0 <= cfg["cap"] <= FF_ABS_CAP[i]:
                    raise ValueError(f"J{i+1}刚度范围0~50、回弹上限0~{FF_ABS_CAP[i]}Nm")
                if not 0 <= cfg["d"] <= 10 or not 0 <= cfg["gravity"] <= 1.3:
                    raise ValueError(f"J{i+1}阻尼0~10、重力比例0~1.3")
                new.append(cfg)
            with lock:
                for i in range(4):
                    settings[i].update(new[i])
                state["status"]="参数及目标角度已更新"
            return True
        except Exception as exc:
            with lock:
                state["status"]="参数错误："+str(exc)
            return False

    def current_as_goals():
        with lock:
            fbs=feedback.copy()
        if any(f is None for f in fbs):
            with lock:
                state["status"]="尚未收到四关节反馈，无法读取当前位置"
            return
        for i,fb in enumerate(fbs):
            target=ANGLE_SIGN[i]*(fb[1]-ZERO_DEG[i])
            edits[i]["goal"].delete(0,"end")
            edits[i]["goal"].insert(0,f"{target:.3f}")
        apply_settings()

    for label,action in (("应用参数 / 目标",apply_settings),("当前位置设为目标",current_as_goals)):
        ttk.Button(control,image=image_label(label,14,BLUE),command=action).pack(side="left",padx=(0,10),ipadx=8,ipady=3)
    capture_var=tk.BooleanVar(value=True)
    ttk.Checkbutton(control,variable=capture_var).pack(side="left",padx=(8,3))
    zh(control,"启动时自动把当前位置设为目标（建议）",13).pack(side="left")

    tablebox=tk.Frame(inner,bg=WHITE,padx=12,pady=11)
    tablebox.pack(fill="x",pady=(0,10))
    zh(tablebox,"实时关节状态与力矩（单位 N·m）",17,color=BLUE).pack(anchor="w",pady=(0,10))
    table=tk.Frame(tablebox,bg=WHITE); table.pack(fill="x")
    columns=["关节","当前角度","目标角度","误差 (°)","速度 (°/s)","理论重力","回弹力矩","阻尼力矩","下发 tff","Iq (mA)","反馈 Hz"]
    for col,h in enumerate(columns):
        zh(table,h,12).grid(row=0,column=col,padx=4,pady=(0,8))
        table.columnconfigure(col,weight=1)
    displays=[]
    for i in range(4):
        zh(table,f"J{i+1}",13,color=BLUE).grid(row=i+1,column=0,padx=5,pady=8)
        v=[]
        for col in range(1,len(columns)):
            var=tk.StringVar(value="--")
            tk.Label(table,textvariable=var,bg=WHITE,fg=TEXT,font=("DejaVu Sans Mono",10),width=12,anchor="center").grid(row=i+1,column=col,padx=1,pady=8)
            v.append(var)
        displays.append(v)

    plotbox=tk.Frame(inner,bg=WHITE,padx=12,pady=9)
    plotbox.pack(fill="x",pady=(0,10))
    zh(plotbox,"最近 12 秒的角度：实线为当前角度，虚线为目标角度",14,color=BLUE).pack(anchor="w")
    graph=tk.Canvas(plotbox,height=192,bg="#172235",highlightthickness=0)
    graph.pack(fill="x",pady=(8,0))

    detail=tk.Frame(inner,bg=BG,pady=4); detail.pack(fill="x",pady=(0,5))
    detail_var=tk.StringVar(value="MIT: 位置=0  速度=0  Kp=0  Kd=0  仅tff | --")
    tk.Label(detail,textvariable=detail_var,bg=BG,fg=GREY,font=("DejaVu Sans Mono",10)).pack(side="left")

    # 底部控制条固定在窗口下方，滚动区域不会挡住停止按钮。
    bottom=tk.Frame(root,bg=BG,padx=20,pady=10)
    bottom.pack(side="bottom",fill="x")
    status_widget=zh(bottom,"未启动",14,bg=BG,color=BLUE)
    status_widget.pack(side="left")
    txvar=tk.StringVar(value="TX: 0")
    tk.Label(bottom,textvariable=txvar,bg=BG,fg=GREY,font=("DejaVu Sans Mono",11)).pack(side="left",padx=16)

    def start():
        global worker
        if worker is not None and worker.is_alive():
            return
        if not apply_settings():
            return
        stop_event.clear()
        with lock:
            for i in range(4):
                feedback[i]=None
                feedback_speed[i]=0.0
                rx_times[i].clear()
                rx_count[i]=0
            history.clear()
            state.update(status="启动中",error="",tx_count=0,rows=[{} for _ in range(4)])
        worker=threading.Thread(target=run_demo if demo else run_real,
                                args=(() if demo else (capture_var.get(),)),daemon=False)
        worker.start()

    def stop():
        stop_event.set()

    btn_stop=ttk.Button(bottom,image=image_label("停止 / 失能",15),command=stop)
    btn_stop.pack(side="right",padx=(10,0),ipadx=12,ipady=6)
    btn_start=ttk.Button(bottom,image=image_label("开始",15),command=start)
    btn_start.pack(side="right",ipadx=15,ipady=6)

    colors=["#FFBF69","#77D3FF","#92E2A5","#FF8FAA"]
    last_status=None
    last_rev=-1

    def refresh():
        nonlocal last_status,last_rev
        with lock:
            status=state["status"]
            rows=[r.copy() for r in state["rows"]]
            rev=state["goal_revision"]
            goals=[p["goal"] for p in settings]
            pts=list(history)
            tx=state["tx_count"]
        if status!=last_status:
            status_widget.config(image=image_label(status,13,BLUE))
            last_status=status
        if rev!=last_rev:
            if last_rev>=0:
                for i,target in enumerate(goals):
                    edits[i]["goal"].delete(0,"end")
                    edits[i]["goal"].insert(0,f"{target:.3f}")
            last_rev=rev
        txvar.set(f"TX: {tx}")
        for i,row in enumerate(rows):
            if not row or "sent" not in row:
                continue
            values=[f"{row['angle']:+.2f}", f"{row['goal']:+.2f}",
                    f"{row['error']:+.2f}",f"{row['speed']:+.1f}",
                    f"{row['gravity']:+.3f}",f"{row['restore']:+.3f}",
                    f"{row['damping']:+.3f}",f"{row['sent']:+.3f}",
                    str(row['iq']),f"{row['rx_hz']:.0f}"]
            for var,value in zip(displays[i],values):
                var.set(value)
        if rows and rows[0] and "sent" in rows[0]:
            r=rows[0]
            detail_var.set(f"MIT P=0 V=0 Kp=0 Kd=0 | J1 tff raw=0x{r['t_raw']:03X} | "
                           f"反馈年龄 {r['rx_age_ms']:.1f}ms")
        graph.delete("all")
        w=max(600,graph.winfo_width()); h=max(190,graph.winfo_height())
        for i in range(4):
            cy=h*(i+0.5)/4
            graph.create_line(40,cy,w-8,cy,fill="#35455B")
            graph.create_text(12,cy,text=f"J{i+1}",anchor="w",fill=colors[i])
        if len(pts)>1:
            end=pts[-1][0]
            valid=[p for p in pts if p[0]>=end-12]
            stride=max(1,len(valid)//250)
            for i in range(4):
                actual_xy=[]; goal_xy=[]
                for t,q,targets in valid[::stride]:
                    x=40+(t-(end-12))/12*(w-55)
                    center=h*(i+0.5)/4
                    magnitude=JOINT_LIMITS[i][1]
                    actual_xy.extend((x,center-clamp(q[i]/magnitude,-1,1)*h/4*0.38))
                    goal_xy.extend((x,center-clamp(targets[i]/magnitude,-1,1)*h/4*0.38))
                if len(actual_xy)>=4:
                    graph.create_line(*actual_xy,fill=colors[i],width=2)
                    graph.create_line(*goal_xy,fill=colors[i],width=1,dash=(3,4))
        alive=worker is not None and worker.is_alive()
        btn_start.config(state="disabled" if alive else "normal")
        root.after(100,refresh)

    def close():
        stop_event.set()
        if worker is not None and worker.is_alive():
            root.after(80,close)
        else:
            root.destroy()
    root.protocol("WM_DELETE_WINDOW",close)
    if demo:
        root.after(120,start)
    refresh()
    root.mainloop()


def self_test():
    # MIT 零物理量字段仍须是中心码，Kp/Kd的原始码才是零。
    for i in range(4):
        p,raw,decoded=mit_payload(0,T_MAX[i])
        assert len(p)==12 and raw==0x800
        assert p.hex().upper()=="CC8000800000000800000000"
        assert abs(decoded)<=T_MAX[i]/4095.0+1e-9
        for torque in (-1.0,0.25,1.0):
            _,_,sent=mit_payload(torque,T_MAX[i])
            assert abs(sent-torque)<=T_MAX[i]/4095.0+1e-9
        buf=(int(30/180*32768)).to_bytes(2,"big",signed=True)+bytes(10)
        assert parse_feedback(0x300|NODE_IDS[i],buf)[0]==i
        assert parse_feedback(NODE_IDS[i],buf)[0]==i
    assert restoring_torque(0,30,10,1)==1.0
    assert restoring_torque(0,-30,10,1)==-1.0
    assert abs(restoring_torque(0,1,10,1)-10*math.radians(1))<1e-10
    params=[dict(goal=30,k=10,cap=1,d=0.5,gravity=1)]*4
    vals=compute_four_joint([0]*4,[0]*4,params,[0.5]*4)
    assert all(abs(v["restore"]-1.0)<1e-10 for v in vals)
    assert all(abs(v["capped_tff"])<=FF_ABS_CAP[i] for i,v in enumerate(vals))
    print("通过：CAN-FD MIT打包、四节点反馈解析、恒力矩限幅、组合重力与阻尼")


def check_model():
    mj,model,data=load_model()
    print("模型：",locate_model(),"自由度 nq/nv:",model.nq,model.nv)
    for q in ([0,0,0,0],[30,20,-20,10],[-30,-20,20,-10]):
        print("q(°)=",q," 重力补偿[Nm]=",[round(x,5) for x in gravity_torques(mj,model,data,q)])
    print("检查完毕；未连接can0、未发送电机指令")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description="四关节恒回弹力矩阻抗控制")
    parser.add_argument("--self-test",action="store_true",help="MIT与控制算法自测，不连接电机")
    parser.add_argument("--check-model",action="store_true",help="仅检查MuJoCo模型")
    parser.add_argument("--demo",action="store_true",help="不连接can0的界面演示")
    parser.add_argument("--model",help="MJCF文件路径，覆盖默认路径")
    args=parser.parse_args()
    if args.model:
        MODEL_PATH=args.model
    if args.self_test:
        self_test()
    elif args.check_model:
        check_model()
    else:
        main(args.demo)
