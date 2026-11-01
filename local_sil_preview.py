"""Passive software-only ROS/Web preview; no ownership or motion is requested."""
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'ros2_ws/src/robot_bringup/test/sil'))
from managed_native_fixture import ManagedNativeFixture, stop
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import subprocess
import yaml
import shutil

running=True
def shutdown(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,shutdown)
signal.signal(signal.SIGINT,shutdown)
h=ManagedNativeFixture(ROOT/'runtime/preview',domain=189)
children=[]
try:
    h.start()
    web=str(Path(get_package_prefix('web_host'))/'lib/web_host')
    share=Path(get_package_share_directory('web_host'))
    static_root=ROOT/'runtime/web'
    if not static_root.exists():shutil.copytree(share/'web',static_root)
    index=static_root/'workbench/index.html'
    html=index.read_text()
    if 'SIL37 纯软件仿真' not in html:
        html=html.replace('<body>','<body><div style="position:fixed;top:0;left:25%;right:10%;z-index:99999;text-align:center;background:#78350f;color:#fef3c7;font:12px sans-serif;padding:4px;pointer-events:none">SIL37 纯软件仿真 · 37轴/标定/总线均为虚构 · 只读 · 未连接真实电机</div>')
        index.write_text(html)
    parameters=ROOT/'runtime/rosbridge.yaml'
    parameters.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
        'address':'127.0.0.1','port':9090,'topics_glob':'[]','topics_pub_glob':'[]',
        'topics_sub_glob':repr(['/robot/state','/motor/*','/joint_states','/tf','/tf_static',
            '/robot/calibration_state','/diagnostics']), 'services_glob':'[]'}}}))
    commands={
      'http':[web+'/static_server','--ros-args','-p','host:=127.0.0.1','-p','port:=8080',
        '-p',f'web_root:={ROOT}/runtime/web','-p','control_mode:=readonly',
        '-p','robot_id:=SIL37-SOFTWARE-ONLY','-p','robot_model:=half_body_robot',
        '-p','model_url_prefix:=/robot_sim','-p',f'model_dir:={get_package_share_directory("robot_description")}'],
      'websocket':[web+'/rosbridge_shim','--ros-args','--params-file',str(parameters)],
    }
    registry=dict(parent=os.getpid(),domain=189,mode='readonly, fictional SIL37, no acquire/arm/enable',
        fixture={k:p.pid for k,p in h.processes.items()},web={})
    for name,command in commands.items():
        log=open(ROOT/'runtime'/f'{name}.log','w')
        process=subprocess.Popen(command,env=h.environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((process,log));registry['web'][name]=process.pid
    (ROOT/'runtime/services.json').write_text(json.dumps(registry,indent=2))
    print(json.dumps(registry),flush=True)
    while running and all(p.poll() is None for p,_ in children) and all(p.poll() is None for p in h.processes.values()):
        time.sleep(.5)
finally:
    for p,log in reversed(children):stop(p);log.close()
    h.close()
