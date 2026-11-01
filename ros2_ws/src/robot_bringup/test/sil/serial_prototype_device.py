"""Native firmware prototype with motor traffic exclusively over XRCE serial."""
import socket
import struct
from pathlib import Path
from rclpy.serialization import serialize_message, deserialize_message
from prototype_device import PrototypeDevice

OUT_TOPICS=['/motor/control_states','/motor/motion_states','/motor/topology',
 '/motor/status_array','/joint_states','/robot_transmission/status','/control/status']
IN_TOPICS=['/motor/policy_array','/control/setpoint','/arm_controller/joint_trajectory','/control/stop']

class _SerialPublisher:
    def __init__(self,link,channel):self.link,self.channel=link,channel
    def publish(self,message):
        data=serialize_message(message)
        try:
            self.link.sock.sendto(struct.pack('<H',self.channel)+data[4:],self.link.peer)
            self.link.tx+=1
        except (BlockingIOError,FileNotFoundError,ConnectionRefusedError):self.link.dropped+=1

class _UnusedPublisher:
    def publish(self,_message):pass

class SerialPrototypeDevice(PrototypeDevice):
    def __init__(self,library,context,driver,local,peer):
        self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        Path(local).unlink(missing_ok=True)
        self.sock.bind(str(local));self.peer=str(peer)
        self.callbacks={};self.tx=self.rx=self.dropped=0
        super().__init__(library,context,driver)
        self.create_timer(.001,self._receive)

    def create_publisher(self,message_type,topic,*args,**kwargs):
        if topic in OUT_TOPICS:return _SerialPublisher(self,OUT_TOPICS.index(topic)+1)
        if message_type.__name__.startswith('Agent'):return _UnusedPublisher()
        return super().create_publisher(message_type,topic,*args,**kwargs)

    def create_subscription(self,message_type,topic,callback,*args,**kwargs):
        if topic in IN_TOPICS:
            self.callbacks[IN_TOPICS.index(topic)+8]=(message_type,callback)
            return None
        if message_type.__name__.startswith('Agent'):return None
        return super().create_subscription(message_type,topic,callback,*args,**kwargs)

    def _receive(self):
        for _ in range(32):
            try:data=self.sock.recv(65536)
            except BlockingIOError:return
            if len(data)<2:continue
            channel=struct.unpack_from('<H',data)[0]
            if channel not in self.callbacks:continue
            message_type,callback=self.callbacks[channel]
            message=deserialize_message(b'\x00\x01\x00\x00'+data[2:],message_type)
            callback(message);self.rx+=1

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()
