"""Standalone local UI prototype; no supervisor, serial, CAN devices or hardware."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix
from prototype_device import PrototypeDevice

root=Path(__file__).resolve().parents[5];folder=root/'evidence/prototype-20261102'
folder.mkdir(parents=True,exist_ok=True)
from prepare_prototype_model import prepare, native_library
model_root=prepare(root,folder/'model-package')
driver=model_root/'models/half_body_robot/config/motor_driver.yaml'
os.environ.update(ROS_DOMAIN_ID='219',ROS_LOCALHOST_ONLY='1')
context=Context();rclpy.init(args=[],context=context,domain_id=219)
library=native_library(root,folder)
node=PrototypeDevice(library,context,driver)
executor=SingleThreadedExecutor(context=context);executor.add_node(node)
web=Path(get_package_prefix('web_host'))/'lib/web_host'
params=folder/'rosbridge.yaml'
params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
 'address':'127.0.0.1','port':9092,'topics_glob':'[]',
 'topics_pub_glob':repr(['/motor/policy_array','/control/setpoint','/arm_controller/joint_trajectory','/control/stop']),
 'topics_sub_glob':repr(['/motor/*','/joint_states','/control/status','/control/trajectory_state','/robot_transmission/status','/imu/*']),
 'services_glob':repr(['/rosapi/*']),'call_services_in_new_thread':True}}}))
commands={
 'http':[str(web/'static_server'),'--ros-args','-p','host:=127.0.0.1','-p','port:=8082','-p','rosbridge_port:=9092',
    '-p',f'web_root:={root}/ros2_ws/src/web_host/web','-p','control_mode:=prototype',
    '-p','robot_id:=LOCAL-SOFTWARE-PROTOTYPE','-p','robot_model:=half_body_robot',
    '-p','model_url_prefix:=/robot_sim','-p',f'model_dir:={model_root}'],
 'websocket':[str(web/'rosbridge_shim'),'--ros-args','--params-file',str(params)]}
children=[]
running=True
def shutdown(*_):
 global running
 running=False
signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
try:
 for name,command in commands.items():
  log=open(folder/f'{name}.log','w')
  p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  children.append((name,p,log))
 (folder/'services.json').write_text(json.dumps({'parent':os.getpid(),'domain':219,'mode':'prototype',
    'processes':{name:p.pid for name,p,_ in children}},indent=2))
 print('Prototype started: http://127.0.0.1:8082/workbench/ ; no control ownership required',flush=True)
 while running and all(p.poll() is None for _,p,_ in children): executor.spin_once(timeout_sec=.002)
finally:
 for _,p,log in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
  log.close()
 executor.shutdown();node.destroy_node();context.shutdown()
