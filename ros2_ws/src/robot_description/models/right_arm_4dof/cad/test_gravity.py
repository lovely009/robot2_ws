#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
4.4 四关节机械臂：MuJoCo 联合重力补偿 + SocketCAN(can0) + 中文界面。

纯 MIT 前馈：P=0、V=0、Kp=0、Kd=0，只调整 tff。
电机 ID 1/2/3/4；J1 T_MAX=50，J2~J4 T_MAX=12。
默认补偿比例为0，上电不会突然补偿；请逐轴缓慢调高比例。

安装：pip install mujoco python-can pillow
检查：python3 gravity_hover_4_4_can0.py --check-model
演示：python3 gravity_hover_4_4_can0.py --demo
实机：python3 gravity_hover_4_4_can0.py
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

# -------------------- 根据实机修改 --------------------
CAN_CHANNEL = "can0"
MODEL_PATH = "./urdf/4.4.mjcf.xml"   # 若放在程序同级目录，也自动查找 ./4.4.mjcf.xml
NODES = [1, 2, 3, 4]              # J1为58lite，其余为48lite
T_MAX = [50.0, 12.0, 12.0, 12.0] # MIT前馈力矩编码范围，须与驱动器设置一致
ZERO_DEG = [0.0, 0.0, 0.0, 0.0]   # 实机已标零则保持0
ANGLE_SIGN = [1, 1, 1, 1]       # 反馈角度 -> MJCF角度，方向不对改为-1
TORQUE_SIGN = [1, 1, 1, 1]      # MJCF力矩 -> MIT力矩，方向不对改为-1
TFF_CAP = [10.0, 5.0, 5.0, 5.0] # 软件力矩上限(Nm)，初始按MJCF执行器上限

DT = 0.005                       # 每5ms发送四轴MIT (200Hz)
RX_TIMEOUT = 0.20                # 简单通信超时保护
FF_STEP = 0.03                   # 每轮tff最大变化量，Nm
JOINT_NAMES = [f"Joint_link{i}" for i in range(1, 5)]

# -------------------- 线程共享数据 --------------------
lock = threading.Lock()
stop_event = threading.Event()
worker = None
feedback = [None] * 4          # (时间, 角度deg, RPM, Iq mA, 故障码, 状态字)
rx_stamps = [deque(maxlen=150) for _ in range(4)]
scales = [0.0] * 4             # 用户滑动条，0~1.0
state = {
    "status": "未启动", "tx": 0, "error": "",
    "rows": [{} for _ in range(4)],
}


def clamp(value, lo, hi):
    return min(max(value, lo), hi)


def encode_torque(torque_nm, tmax):
    """MIT 12bit 前馈原始码：0~4095 对应 [-T_MAX,+T_MAX]。"""
    value = clamp(torque_nm, -tmax, tmax)
    return int(math.floor((value + tmax) * 4095.0 / (2 * tmax) + 0.5))


def decode_torque(code, tmax):
    return code * 2 * tmax / 4095.0 - tmax


def mit_payload(tff_nm, tmax, enable=True):
    """12字节 MIT：P=0，V=0，Kp=0，Kd=0，仅tff变化。"""
    p, v, kp, kd = 0x8000, 0x800, 0, 0
    t = encode_torque(tff_nm, tmax)
    payload = bytes([
        0xCC if enable else 0x0C,
        p >> 8, p & 255,
        v >> 4, ((v & 15) << 4) | (kp >> 8), kp & 255,
        kd >> 4, ((kd & 15) << 4) | (t >> 8), t & 255,
        0, 0, 0,
    ])
    return payload, t, decode_torque(t, tmax)


def decode_feedback(can_id, payload):
    """反馈帧：优先0x301~0x304，兼容设备实际直接使用0x001~0x004。"""
    if len(payload) != 12:
        return None
    for i, node in enumerate(NODES):
        if can_id in (0x300 | node, node):
            angle = int.from_bytes(payload[0:2], "big", signed=True) * 180.0 / 32768.0
            rpm = int.from_bytes(payload[2:4], "big", signed=True)
            iq = int.from_bytes(payload[4:6], "big", signed=True)
            fault = int.from_bytes(payload[6:8], "big")
            status = payload[11]
            return i, (time.monotonic(), angle, rpm, iq, fault, status)
    return None


def send_mit(bus, can_module, i, torque_nm, enable=True):
    payload, raw, actual = mit_payload(torque_nm, T_MAX[i], enable)
    frame = can_module.Message(
        arbitration_id=0x110 | NODES[i],
        data=payload, is_extended_id=False,
        is_fd=True, bitrate_switch=True, check=True,
    )
    bus.send(frame, timeout=0.03)
    return raw, actual


def receive_loop(bus):
    """独立线程不断接收电机自动回复，不发送查询帧。"""
    try:
        while not stop_event.is_set():
            frame = bus.recv(timeout=0.03)
            if frame is None or frame.is_error_frame or frame.is_extended_id:
                continue
            item = decode_feedback(frame.arbitration_id, bytes(frame.data))
            if item is None:
                continue
            i, fb = item
            with lock:
                feedback[i] = fb
                rx_stamps[i].append(fb[0])
    except Exception as exc:
        with lock:
            state["error"] = f"接收线程异常：{exc}"


def model_file():
    for candidate in (MODEL_PATH, "./4.4.mjcf.xml", "./urdf/4.4.xml"):
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f"找不到MJCF模型：{MODEL_PATH}，请修改MODEL_PATH")


def load_mujoco():
    """加载上传的MJCF；如果缺少STL，只移除显示网格，保留全部惯性与关节。"""
    import mujoco
    path = model_file()
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as exc:
        if not any(k in str(exc).lower() for k in ("mesh", "file", "resource", "asset", "stl")):
            raise
        root = ET.parse(path).getroot()
        for parent in root.iter():
            for child in list(parent):
                if child.tag in ("asset", "geom"):
                    parent.remove(child)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        print("未找到部分STL，已只使用MJCF中的质量、质心、惯量和关节计算重力。")
    if model.nq != 4 or model.nv != 4:
        raise RuntimeError(f"模型应有4个自由度，实际nq={model.nq}, nv={model.nv}")
    for i, name in enumerate(JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0 or model.jnt_qposadr[jid] != i or model.jnt_dofadr[jid] != i:
            raise RuntimeError(f"MJCF关节顺序不符合预期：{name}")
    return mujoco, model, mujoco.MjData(model)


def gravity_all(mujoco, model, data, angles_deg):
    """四关节联合计算：dq=0时qfrc_bias就是静态重力补偿力矩。"""
    data.qpos[:] = [math.radians(a) for a in angles_deg]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    return [float(x) for x in data.qfrc_bias]


def get_hz(stamps):
    if len(stamps) < 2:
        return 0.0
    interval = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / interval if interval > 0 else 0.0


def run_real():
    """主控制线程：读取四轴角度 -> MuJoCo重力 -> 下发各轴tff。"""
    bus = None
    receiver = None
    prev_ff = [0.0] * 4
    try:
        import can
        mj, model, data = load_mujoco()
        bus = can.Bus(interface="socketcan", channel=CAN_CHANNEL,
                      fd=True, receive_own_messages=False)
        receiver = threading.Thread(target=receive_loop, args=(bus,), daemon=True)
        receiver.start()
        start = time.monotonic()
        next_tick = start
        tx = 0
        with lock:
            state["status"] = "正在发送零力矩MIT，等待四台电机反馈"
        while not stop_event.is_set():
            now = time.monotonic()
            with lock:
                fbs = feedback.copy()
                gains = scales.copy()
                rx_copy = [list(x) for x in rx_stamps]
                rx_error = state["error"]
            if rx_error:
                raise RuntimeError(rx_error)

            ready = all(x is not None for x in fbs)
            if not ready and now - start > 3.0:
                missing = [str(NODES[i]) for i, x in enumerate(fbs) if x is None]
                raise TimeoutError("等待反馈超时：电机ID " + ",".join(missing))
            if ready and any(now - x[0] > RX_TIMEOUT for x in fbs):
                missing = [str(NODES[i]) for i, x in enumerate(fbs) if now - x[0] > RX_TIMEOUT]
                raise TimeoutError("反馈超时：电机ID " + ",".join(missing))
            if ready and any(x[5] & 0x20 for x in fbs):
                raise RuntimeError("电机报告故障：" + ",".join(
                    str(NODES[i]) for i, x in enumerate(fbs) if x[5] & 0x20))

            if ready:
                model_deg = [ANGLE_SIGN[i] * (fbs[i][1] - ZERO_DEG[i]) for i in range(4)]
                g = gravity_all(mj, model, data, model_deg)
            else:
                model_deg, g = [0.0] * 4, [0.0] * 4

            rows = []
            for i in range(4):
                target = TORQUE_SIGN[i] * g[i] * gains[i] if ready else 0.0
                target = clamp(target, -TFF_CAP[i], TFF_CAP[i])
                prev_ff[i] += clamp(target - prev_ff[i], -FF_STEP, FF_STEP)
                raw, sent = send_mit(bus, can, i, prev_ff[i])
                tx += 1
                if ready:
                    f = fbs[i]
                    rows.append({
                        "angle": f[1], "g": g[i], "sent": sent,
                        "raw": raw, "iq": f[3],
                        "hz": get_hz(rx_copy[i]), "age": (now - f[0]) * 1000,
                    })
                else:
                    rows.append({})
            with lock:
                state["tx"] = tx
                state["rows"] = rows
                state["status"] = "四轴重力补偿运行中" if ready else "等待四轴反馈"

            next_tick += DT
            wait = next_tick - time.monotonic()
            if wait < -DT:
                next_tick = time.monotonic()
            else:
                stop_event.wait(max(0.0, wait))
    except Exception as exc:
        print("控制停止原因：", exc, flush=True)
        with lock:
            state["status"] = f"停止：{exc}"
    finally:
        stop_event.set()
        if bus is not None:
            for i in range(4):
                try:
                    send_mit(bus, can, i, 0.0, enable=False)
                except Exception:
                    pass
            if receiver is not None:
                receiver.join(timeout=0.2)
            bus.shutdown()
        with lock:
            if not state["status"].startswith("停止："):
                state["status"] = "已停止，已请求四轴失能"


def run_demo():
    """只演示界面，不连接can0，不发送电机指令。"""
    try:
        mj, model, data = load_mujoco()
        begin = time.monotonic()
        while not stop_event.is_set():
            t = time.monotonic() - begin
            deg = [20 * math.sin(t * 0.3), 30 * math.sin(t * 0.4),
                   25 * math.sin(t * 0.25), 15 * math.sin(t * 0.36)]
            g = gravity_all(mj, model, data, deg)
            with lock:
                gains = scales.copy()
            rows = []
            for i in range(4):
                cmd = clamp(TORQUE_SIGN[i] * g[i] * gains[i], -TFF_CAP[i], TFF_CAP[i])
                raw = encode_torque(cmd, T_MAX[i])
                rows.append({"angle": deg[i], "g": g[i], "sent": decode_torque(raw, T_MAX[i]),
                             "raw": raw, "iq": 0, "hz": 0, "age": 0})
            with lock:
                state["status"] = "模拟演示（没有连接电机）"
                state["rows"] = rows
            stop_event.wait(0.04)
    except Exception as exc:
        with lock:
            state["status"] = f"演示失败：{exc}"


def chinese_font():
    """用Pillow画中文，避免部分WSL/Tk中文被显示为\\uXXXX。"""
    try:
        out = subprocess.check_output(
            ["fc-match", "-f", "%{file}\n", "Noto Sans CJK SC:lang=zh-cn"],
            text=True, timeout=3).splitlines()
        if out and os.path.isfile(out[0]):
            return out[0]
    except (OSError, subprocess.SubprocessError):
        pass
    for name in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                 "C:/Windows/Fonts/msyh.ttc"):
        if os.path.isfile(name):
            return name
    raise RuntimeError("没有中文字体，请安装：sudo apt install fonts-noto-cjk")


def main(demo=False):
    global worker
    from PIL import Image, ImageDraw, ImageFont, ImageTk
    root = tk.Tk()
    root.title("四轴机械臂重力补偿")
    root.geometry("1200x450")
    root.configure(bg="#F4F7FB")
    font_path = chinese_font()
    image_cache = {}
    fonts = {}

    def zh_img(text, size=15, color="#24354C"):
        key = (str(text), size, color)
        if key not in image_cache:
            if size not in fonts:
                fonts[size] = ImageFont.truetype(font_path, size)
            font = fonts[size]
            bb = font.getbbox(str(text))
            img = Image.new("RGBA", (max(1, bb[2]-bb[0]+10), max(1, bb[3]-bb[1]+10)))
            ImageDraw.Draw(img).text((5-bb[0], 5-bb[1]), str(text), font=font, fill=color)
            image_cache[key] = ImageTk.PhotoImage(img, master=root)
        return image_cache[key]

    def zh_label(parent, text, size=15, bg="white", color="#24354C"):
        return tk.Label(parent, image=zh_img(text, size, color), bg=bg, bd=0)

    heading = tk.Frame(root, bg="#F4F7FB", pady=12)
    heading.pack(fill="x", padx=20)
    zh_label(heading, "4.4 机械臂 · 四关节重力补偿", 24, bg="#F4F7FB").pack(anchor="w")
    zh_label(heading, "SocketCAN can0 · 每轴 MIT 只发送前馈 tff · 初始补偿为 0%", 14,
             bg="#F4F7FB", color="#526779").pack(anchor="w")

    panel = tk.Frame(root, bg="white", padx=14, pady=15)
    panel.pack(fill="x", padx=20, pady=(0, 10))
    headers = ["关节", "角度 (°)", "理论重力 (Nm)", "下发力矩 (Nm)",
               "tff原始码", "反馈Iq (mA)", "反馈频率", "反馈年龄", "补偿比例"]
    for c, label in enumerate(headers):
        zh_label(panel, label, 13).grid(row=0, column=c, padx=9, pady=(0, 12))

    fields = []
    for i in range(4):
        values = []
        zh_label(panel, f"J{i+1}  {'58' if i == 0 else '48'}", 15).grid(row=i+1, column=0, padx=8, pady=9)
        for col in range(1, 8):
            value = tk.StringVar(value="--")
            tk.Label(panel, textvariable=value, bg="white", fg="#1154A0",
                     font=("DejaVu Sans Mono", 11), width=12).grid(
                row=i+1, column=col, padx=2, pady=9)
            values.append(value)
        f = tk.Frame(panel, bg="white")
        f.grid(row=i+1, column=8, padx=5)
        s = ttk.Scale(f, from_=0, to=100, length=110)
        s.set(0)
        s.pack(side="left")
        pct = tk.StringVar(value="0%")
        tk.Label(f, textvariable=pct, width=5, bg="white").pack(side="left")

        def on_gain(value, index=i, view=pct):
            percent = float(value)
            with lock:
                scales[index] = percent / 100.0
            view.set(f"{percent:.0f}%")
        s.configure(command=on_gain)
        fields.append(values)

    controls = tk.Frame(root, bg="#F4F7FB")
    controls.pack(fill="x", padx=20, pady=8)
    status_lbl = zh_label(controls, "未启动", 15, bg="#F4F7FB", color="#1457A2")
    status_lbl.pack(side="left")
    count_label = tk.StringVar(value="TX: 0")
    tk.Label(controls, textvariable=count_label, bg="#F4F7FB").pack(side="left", padx=20)

    def start():
        global worker
        if worker is not None and worker.is_alive():
            return
        with lock:
            state.update(status="正在启动", error="", tx=0, rows=[{} for _ in range(4)])
            for i in range(4):
                feedback[i] = None
                rx_stamps[i].clear()
        stop_event.clear()
        worker = threading.Thread(target=run_demo if demo else run_real, daemon=False)
        worker.start()

    def stop():
        stop_event.set()

    btn_start = ttk.Button(controls, image=zh_img("开始", 17), command=start)
    btn_start.pack(side="right", padx=8)
    btn_stop = ttk.Button(controls, image=zh_img("停止 / 失能", 17), command=stop)
    btn_stop.pack(side="right", padx=8)

    zh_label(root, "注意：下发力矩是MIT命令值，不是扭矩传感器实测值。纯重力补偿不保证绝对不漂移。",
             13, bg="#F4F7FB", color="#92551C").pack(anchor="w", padx=20)

    last_status = None
    def refresh():
        nonlocal last_status
        with lock:
            snapshot = dict(state)
            rows = [x.copy() for x in state["rows"]]
        if snapshot["status"] != last_status:
            last_status = snapshot["status"]
            status_lbl.configure(image=zh_img(last_status, 15, "#1457A2"))
        count_label.set(f"TX: {snapshot['tx']}")
        for i, row in enumerate(rows):
            if not row:
                continue
            values = [f"{row['angle']:+.2f}", f"{row['g']:+.3f}",
                      f"{row['sent']:+.3f}", f"0x{row['raw']:03X}",
                      str(row['iq']), f"{row['hz']:.0f} Hz", f"{row['age']:.1f} ms"]
            for widget, value in zip(fields[i], values):
                widget.set(value)
        btn_start.configure(state="disabled" if worker is not None and worker.is_alive() else "normal")
        root.after(100, refresh)

    def close():
        stop_event.set()
        if worker is not None and worker.is_alive():
            root.after(100, close)
        else:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    if demo:
        root.after(120, start)
    refresh()
    root.mainloop()


def self_test():
    payload, raw, sent = mit_payload(0.0, 50.0)
    assert len(payload) == 12 and raw == 0x800
    assert payload.hex().upper() == "CC8000800000000800000000"
    for i in range(4):
        p, code, val = mit_payload(1.0, T_MAX[i])
        assert len(p) == 12 and abs(val - 1.0) < 0.02
        fb = (int(30 / 180 * 32768)).to_bytes(2, "big", signed=True) + bytes(10)
        assert decode_feedback(0x300 | NODES[i], fb)[0] == i
    print("自检通过：4台电机ID、MIT打包、零力矩编码、反馈解析")


def check_model():
    mj, model, data = load_mujoco()
    print("MJCF模型：", model_file())
    print("关节：", JOINT_NAMES)
    for q in ([0, 0, 0, 0], [30, 15, -20, 10], [-30, -15, 20, -10]):
        print("角度", q, "-> 重力补偿 Nm", [round(v, 4) for v in gravity_all(mj, model, data, q)])
    print("仅检查模型，没有连接电机")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="不连接电机，显示模拟数据")
    parser.add_argument("--check-model", action="store_true", help="只检查MJCF重力计算")
    parser.add_argument("--self-test", action="store_true", help="不连接电机，自检MIT协议")
    parser.add_argument("--model", help="覆盖MJCF文件位置")
    args = parser.parse_args()
    if args.model:
        MODEL_PATH = args.model
    if args.self_test:
        self_test()
    elif args.check_model:
        check_model()
    else:
        main(demo=args.demo)
