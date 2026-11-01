"""Real PTY + micro-ROS Agent + native firmware prototype, isolated from the other UI."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[5];folder=root/'evidence/virtual-mcu-20261102'
folder.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'ros2_ws/src/robot_bringup/test/sil'))
os.environ.update(ROS_DOMAIN_ID='221',ROS_LOCALHOST_ONLY='1', FASTRTPS_DEFAULT_PROFILES_FILE=str(Path(__file__).parent/'serial/fastdds-loopback.xml'))
from serial_prototype_device import SerialPrototypeDevice
from build_serial_gateway import build
build(root,folder)
from prepare_prototype_model import prepare, native_library
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
model_root=prepare(root,root/'evidence/prototype-20261102/model-package')
local,peer=folder/'device.sock',folder/'xrce.sock'
context=Context();rclpy.init(args=[],context=context,domain_id=221)
node=SerialPrototypeDevice(native_library(root,folder),context,
    model_root/'models/half_body_robot/config/motor_driver.yaml',local,peer)
executor=SingleThreadedExecutor(context=context);executor.add_node(node)
web=Path(get_package_prefix('web_host'))/'lib/web_host'
params=folder/'rosbridge.yaml'
params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
 'address':'127.0.0.1','port':9093,'topics_glob':'[]',
 'topics_pub_glob':repr(['/motor/policy_array','/control/setpoint','/arm_controller/joint_trajectory','/control/stop']),
 'topics_sub_glob':repr(['/motor/*','/joint_states','/control/status','/robot_transmission/status']),
 'services_glob':repr(['/rosapi/*']),'call_services_in_new_thread':True}}}))
children=[];running=True
firmware_path='/tmp/robot2-virtual-firmware';agent_path='/tmp/robot2-virtual-mcu'
for path in [firmware_path,agent_path]:
    if Path(path).is_symlink():Path(path).unlink()
def launch(name,command):
    log=open(folder/f'{name}.log','w')
    p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((name,p,log));return p

def shutdown(*_):
    global running
    running=False
signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
try:
    launch('pty',['socat','-d','-d',f'pty,raw,echo=0,link={firmware_path}',f'pty,raw,echo=0,link={agent_path}'])
    deadline=time.monotonic()+5
    while not Path(agent_path).exists():
        if time.monotonic()>deadline:raise RuntimeError('PTY creation timed out')
        time.sleep(.02)
    launch('agent',[str(root/'micro_ros_agent/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent'),
        'serial','--dev',agent_path,'-b','115200','-v','4'])
    launch('xrce',[str(folder/'gateway-build/xrce_serial_gateway'),firmware_path,'221',str(peer),str(local)])
    launch('http',[str(web/'static_server'),'--ros-args','-p','host:=127.0.0.1','-p','port:=8083','-p','rosbridge_port:=9093',
        '-p',f'web_root:={root}/ros2_ws/src/web_host/web','-p','control_mode:=prototype',
        '-p','robot_id:=VIRTUAL-SERIAL-SOFTWARE','-p','robot_model:=half_body_robot',
        '-p','model_url_prefix:=/robot_sim','-p',f'model_dir:={model_root}'])
    launch('websocket',[str(web/'rosbridge_shim'),'--ros-args','--params-file',str(params)])
    (folder/'services.json').write_text(json.dumps({'parent':os.getpid(),'domain':221,'mode':'prototype-over-XRCE-serial',
        'firmware_serial':firmware_path,'agent_serial':agent_path,'processes':{n:p.pid for n,p,_ in children}},indent=2))
    print('Virtual serial MCU: http://127.0.0.1:8083/workbench/ ; ws://127.0.0.1:9093',flush=True)
    node.create_timer(1.,lambda:(folder/'serial-counts.json').write_text(json.dumps({'to_xrce':node.tx,'from_xrce':node.rx,'startup_drops':node.dropped})))
    while running and all(p.poll() is None for _,p,_ in children):executor.spin_once(timeout_sec=.002)
finally:
    (folder/'serial-counts.json').write_text(json.dumps({'to_xrce':node.tx,'from_xrce':node.rx,'startup_drops':node.dropped}))
    for _,p,log in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        log.close()
    executor.shutdown();node.destroy_node();context.shutdown()
