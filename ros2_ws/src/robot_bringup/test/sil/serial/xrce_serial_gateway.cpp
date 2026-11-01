// Real XRCE serial client; local datagrams only join native firmware to ROS CDR codecs.
#include <uxr/client/client.h>
#include <ucdr/microcdr.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

struct Topic {const char* name;const char* type;bool output;};
static const Topic topics[]={
 {"/motor/control_states","can_bridge_msgs::msg::dds_::MotorControlStateArray_",true},
 {"/motor/motion_states","can_bridge_msgs::msg::dds_::MotorMotionStateArray_",true},
 {"/motor/topology","can_bridge_msgs::msg::dds_::MotorTopology_",true},
 {"/motor/status_array","can_bridge_msgs::msg::dds_::MotorStatusArray_",true},
 {"/joint_states","sensor_msgs::msg::dds_::JointState_",true},
 {"/robot_transmission/status","std_msgs::msg::dds_::String_",true},
 {"/control/status","std_msgs::msg::dds_::String_",true},
 {"/motor/policy_array","can_bridge_msgs::msg::dds_::MotorPolicyArray_",false},
 {"/control/setpoint","can_bridge_msgs::msg::dds_::MotorCommandArray_",false},
 {"/arm_controller/joint_trajectory","trajectory_msgs::msg::dds_::JointTrajectory_",false},
 {"/control/stop","std_msgs::msg::dds_::Empty_",false},
};
struct Local {int fd;sockaddr_un peer;uint64_t received=0;};
static void callback(uxrSession*,uxrObjectId object,uint16_t,uxrStreamId,ucdrBuffer* ub,uint16_t length,void* arg){
 auto &local=*static_cast<Local*>(arg);
 std::vector<uint8_t> data(length+2);data[0]=object.id&255;data[1]=object.id>>8;
 if(!ucdr_deserialize_array_uint8_t(ub,data.data()+2,length))return;
 if(sendto(local.fd,data.data(),data.size(),0,reinterpret_cast<sockaddr*>(&local.peer),sizeof(local.peer))>=0)++local.received;
}
static bool status(uxrSession& session,uint16_t request){
 uint8_t result=255;
 bool ok=uxr_run_session_until_all_status(&session,3000,&request,&result,1);
 if(!ok)std::fprintf(stderr,"XRCE create failed status=%u\n",result);
 return ok;
}
int main(int argc,char** argv){
 if(argc!=5){std::fprintf(stderr,"usage: %s serial-device domain local-socket peer-socket\n",argv[0]);return 2;}
 Local local{};local.fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0);
 sockaddr_un address{};address.sun_family=AF_UNIX;local.peer.sun_family=AF_UNIX;
 if(std::strlen(argv[3])>=sizeof(address.sun_path)||std::strlen(argv[4])>=sizeof(local.peer.sun_path))return 2;
 std::strcpy(address.sun_path,argv[3]);std::strcpy(local.peer.sun_path,argv[4]);unlink(argv[3]);
 if(bind(local.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0){perror("bind local socket");return 2;}
 int fd=open(argv[1],O_RDWR|O_NOCTTY|O_NONBLOCK);
 uxrSerialTransport transport;
 if(fd<0||!uxr_init_serial_transport(&transport,fd,0,1)){perror("serial transport");return 2;}
 uxrSession session;uxr_init_session(&session,&transport.comm,0x52423232);
 uxr_set_topic_callback(&session,callback,&local);
 if(!uxr_create_session(&session)){std::fprintf(stderr,"XRCE serial session failed\n");return 2;}
 static uint8_t out_buffer[512*32],in_buffer[512*32];
 auto out=uxr_create_output_reliable_stream(&session,out_buffer,sizeof(out_buffer),32);
 auto in=uxr_create_input_reliable_stream(&session,in_buffer,sizeof(in_buffer),32);
 auto participant=uxr_object_id(1,UXR_PARTICIPANT_ID);
 const char *xml="<dds><participant><rtps><name>virtual_firmware_serial</name><userTransports><transport_id>virtual_serial_loopback</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></dds>";
 if(!status(session,uxr_buffer_create_participant_xml(&session,out,participant,std::atoi(argv[2]),xml,UXR_REPLACE)))return 3;
 auto publisher=uxr_object_id(1,UXR_PUBLISHER_ID),subscriber=uxr_object_id(1,UXR_SUBSCRIBER_ID);
 if(!status(session,uxr_buffer_create_publisher_xml(&session,out,publisher,participant,"",UXR_REPLACE)))return 3;
 if(!status(session,uxr_buffer_create_subscriber_xml(&session,out,subscriber,participant,"",UXR_REPLACE)))return 3;
 for(unsigned i=0;i<sizeof(topics)/sizeof(topics[0]);++i){
  const auto &t=topics[i];unsigned id=i+1;
  std::string body="<topic><kind>NO_KEY</kind><name>rt"+std::string(t.name)+"</name><dataType>"+t.type+"</dataType></topic>";
  std::string topic_xml="<dds>"+body+"</dds>";
  if(!status(session,uxr_buffer_create_topic_xml(&session,out,uxr_object_id(id,UXR_TOPIC_ID),participant,topic_xml.c_str(),UXR_REPLACE)))return 3;
  const char* kind=t.output?"data_writer":"data_reader";
  std::string qos="<qos><reliability><kind>"+std::string(t.output?"RELIABLE":"BEST_EFFORT")+"</kind></reliability></qos>";
  std::string entity="<dds><"+std::string(kind)+"><historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"+qos+body+"</"+kind+"></dds>";
  auto object=uxr_object_id(id,t.output?UXR_DATAWRITER_ID:UXR_DATAREADER_ID);
  auto request=t.output?uxr_buffer_create_datawriter_xml(&session,out,object,publisher,entity.c_str(),UXR_REPLACE):
      uxr_buffer_create_datareader_xml(&session,out,object,subscriber,entity.c_str(),UXR_REPLACE);
  if(!status(session,request))return 3;
  if(!t.output){uxrDeliveryControl delivery{};delivery.max_samples=UXR_MAX_SAMPLES_UNLIMITED;uxr_buffer_request_data(&session,out,object,in,&delivery);}
 }
 std::puts("XRCE serial session and 11 ROS topic entities ready");std::fflush(stdout);
 uint64_t sent=0,dropped=0;uint8_t bytes[65536];unsigned iteration=0;
 while(true){
  for(unsigned batch=0;batch<12;++batch){
   ssize_t size=recv(local.fd,bytes,sizeof(bytes),0);
   if(size<0)break;if(size<2)continue;
   unsigned id=bytes[0]+(unsigned(bytes[1])<<8);
   if(!id||id>7)continue;
   ucdrBuffer buffer;
   if(uxr_prepare_output_stream(&session,out,uxr_object_id(id,UXR_DATAWRITER_ID),&buffer,size-2)){
    ucdr_serialize_array_uint8_t(&buffer,bytes+2,size-2);++sent;
   }else ++dropped;
  }
  uxr_run_session_time(&session,2);
  if(++iteration%2500==0){std::printf("serial samples tx=%llu rx=%llu local_drop=%llu\n",(unsigned long long)sent,(unsigned long long)local.received,(unsigned long long)dropped);std::fflush(stdout);}
 }
}
