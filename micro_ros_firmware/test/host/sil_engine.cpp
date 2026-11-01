// Native SIL transport adapter. Executes production gate/kernel/protocol/router;
// only the RTOS clock, boot device inventory and physical devices are fixtures.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <chrono>
#include "agent_control_gate.h"
#include "motor_control_internal.h"
#include "motor_tx_router.h"

namespace {
constexpr unsigned N=37;
struct OutputEvent {uint64_t wall_ns; uint32_t tick; uint32_t motor_id;};
// Measurement-only fixed storage: no allocation or Python callbacks in the
// 200Hz output path. Tick time and actual host pump time remain separate.
constexpr unsigned OUTPUT_CAPACITY=N*200*60;
OutputEvent output_events[OUTPUT_CAPACITY];
unsigned output_count=0;
unsigned output_overflow=0;
struct Device {double position=0, velocity=0, target=0; bool enabled=false; unsigned outputs=0;};
struct Engine {
  motor_control_context_t core{};
  control_mailbox_t mailbox{};
  agent_control_gate_t gate{};
  motor_tx_router_t router{};
  can_tx_scheduler_t scheduler[3]{};
  Device device[N];
  unsigned tick=1000;
  Engine() {
    const motor_protocol_adapter_t *adapters[]={&motor_protocol_cerb_adapter};
    // Registry remains static for the lifetime of this harness.
    static const motor_protocol_adapter_t *registry[]={&motor_protocol_cerb_adapter};
    (void)adapters;
    motor_control_init(&core, registry, 1);
    control_mailbox_init(&mailbox);
    agent_gate_begin_session(&gate,&mailbox,100);
    control_mailbox_request_agent_connected(&mailbox,true);
    motor_tx_router_init(&router);
    for(auto &s:scheduler)can_tx_scheduler_init(&s);
    core.entry_count=N;
    for(unsigned i=0;i<N;++i) {
      auto &e=core.entries[i]; e.used=e.online=e.has_measured=true;
      e.motor_id=e.node=i+1; e.bus_index=i%3; e.adapter=&motor_protocol_cerb_adapter;
      for(unsigned p=0;p<motor_control_profile_count();++p) {
        auto profile=motor_control_profile_at(p);
        if(std::strcmp(profile->model_name,"cerb_r48_101")==0)e.profile=profile;
      }
      e.lifecycle=MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
      e.identity_read_done=e.dev_read_done=e.boot_record_offered=true;
      e.boot_norm_stage=MOTOR_CONTROL_BOOT_NORM_DONE;
      e.dev_init=MOTOR_CONTROL_DEV_INIT_OK;
      e.dev_raw_present=0xff; e.dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_T_MAX]=300;
      e.last_motion_rx_tick_ms=e.last_rx_tick_ms=tick;
      // No policy, motion authorization or driver enable is fabricated here.
    }
  }
  void receive(unsigned id,const uint8_t *data) {
    if(!id||id>N)return;
    auto &d=device[id-1];
    d.enabled=(data[0]&0x80)!=0;
    if(d.enabled) {
      auto raw=unsigned(data[1])*256+data[2];
      auto limit=core.entries[id-1].profile->mit_position_limit_rad;
      d.target=(double(raw)/65535*2-1)*limit; ++d.outputs;
      if(output_count<OUTPUT_CAPACITY) {
        const auto now=std::chrono::steady_clock::now().time_since_epoch();
        output_events[output_count++]={uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()),tick,id};
      } else ++output_overflow;
    } else d.velocity=0;
  }
  static bool tx(void *ctx,uint8_t bus,const can_tx_frame_t *frame) {
    auto &e=*static_cast<Engine*>(ctx); (void)bus;
    if(frame->can_id==MOTOR_PROTOCOL_CERB_MULTI_COMMAND_ID) {
      for(unsigned i=0;i<6&&frame->data[56+i];++i)e.receive(frame->data[56+i],frame->data+i*9);
    } else if(frame->can_id>0x110&&frame->can_id<=0x110+N)e.receive(frame->can_id-0x110,frame->data);
    return true;
  }
  void feedback() {
    for(unsigned i=0;i<N;++i) {
      const auto &d=device[i]; const auto &e=core.entries[i];
      int16_t p=std::lround(d.position*180/MOTOR_PROTOCOL_PI*182);
      int16_t v=std::lround(d.velocity*e.profile->gear_ratio*60/(2*MOTOR_PROTOCOL_PI));
      motor_protocol_raw_frame_t f{};f.bus_index=i%3;f.can_id=0x301+i;
      f.is_fd=f.is_brs=true;f.dlc=12;f.rx_tick_ms=tick;
      f.data[0]=uint16_t(p)>>8;f.data[1]=p;f.data[2]=uint16_t(v)>>8;f.data[3]=v;
      motor_control_on_raw_frame(&core,&f);
    }
  }
  void step() {
    ++tick;
    for(auto &d:device) {
      d.velocity=d.enabled?std::clamp((d.target-d.position)*30.,-2.,2.):0.;
      d.position+=d.velocity*.001;
    }
    if(tick%20==0)feedback();
    agent_gate_service(&gate,&mailbox,&core,tick,true);
    agent_gate_admit_command(&gate,&mailbox,&core,tick);
    if(tick%5==0) {
      agent_gate_step(&gate,&mailbox,&core,tick,true);
      motor_control_hold_step(&core,tick);
    }
    motor_control_service(&core,tick);
    auto canceled=motor_control_take_tx_cancellations(&core);
    for(auto &s:scheduler)can_tx_scheduler_cancel_slots(&s,canceled);
    motor_protocol_tx_frame_t frames[MOTOR_TX_ROUTER_LOGICAL_CAPACITY];
    auto count=motor_control_collect_tx(&core,frames,MOTOR_TX_ROUTER_LOGICAL_CAPACITY);
    for(unsigned i=0;i<count;++i) {
      const auto &f=frames[i];can_tx_frame_t out{};
      out.can_id=f.can_id;out.tx_slot=f.tx_slot;out.node=f.node;out.aggregate_only=f.aggregate_only;
      out.is_extended=f.is_extended;out.is_fd=f.is_fd;out.is_brs=f.is_brs;out.size=f.size;
      std::memcpy(out.data,f.data,f.size);
      if(f.tx_class==MOTOR_PROTOCOL_TX_CLASS_P0) {
        if(!can_tx_scheduler_submit_p0(&scheduler[f.bus_index],&out))motor_control_requeue_p0(&core,&f);
      } else if(f.tx_class==MOTOR_PROTOCOL_TX_CLASS_P1)can_tx_scheduler_submit_p1(&scheduler[f.bus_index],f.tx_slot,&out);
      else can_tx_scheduler_submit_p2(&scheduler[f.bus_index],&out);
    }
    for(unsigned b=0;b<3;++b)motor_tx_router_pump(&router,&scheduler[b],b,tx,this);
  }
};
Engine *engine=nullptr;
}
extern "C" {
void sil_init(){delete engine;engine=new Engine;output_count=output_overflow=0;}
unsigned sil_output_events(OutputEvent *destination,unsigned capacity) {
 const auto count=std::min(capacity,output_count);
 std::memcpy(destination,output_events,count*sizeof(OutputEvent));return count;
}
unsigned sil_output_overflow(){return output_overflow;}
void sil_advance(uint32_t tick){while(engine->tick<tick)engine->step();}
int sil_lease(uint64_t id,uint32_t seq,uint32_t deadline,int active){return control_lease_submit(&engine->gate.authority,100,id,seq,deadline,active,engine->tick);}
int sil_command(uint64_t lease,uint32_t seq,uint32_t deadline,unsigned count,const uint8_t*ids,const int16_t*p,const int16_t*v,const int16_t*t){
 control_command_snapshot_t m{};m.session_id=100;m.lease_id=lease;m.sequence=seq;
 m.received_tick_ms=engine->tick;m.valid_until_tick_ms=deadline;m.target_count=count;
 for(unsigned i=0;i<count&&i<N;++i)m.targets[i]={ids[i],float(wire_dequantize(p[i],12.5)),float(wire_dequantize(v[i],64)),float(wire_dequantize(t[i],64))};
 return agent_gate_submit_command(&engine->gate,&engine->mailbox,&m,engine->tick);
}
int sil_policy(uint64_t lease,uint32_t req,uint32_t gen,uint32_t deadline,unsigned count,const uint16_t*ids,const uint8_t*enabled,const float*kp,const float*kd){
 control_policy_snapshot_t m{};m.lease_id=lease;m.request_id=req;m.policy_generation=gen;
 m.received_tick_ms=engine->tick;m.valid_until_tick_ms=deadline;m.target_count=count;
 for(unsigned i=0;i<count&&i<N;++i)m.targets[i]={ids[i],bool(enabled[i]),kp[i],kd[i],false};
 return agent_gate_submit_policy(&engine->gate,&engine->mailbox,&m,engine->tick);
}
void sil_stop(){agent_gate_request_stop(&engine->gate);}
uint64_t sil_enabled_mask(){uint64_t mask=0;for(unsigned i=0;i<N;++i)if(engine->device[i].enabled)mask|=UINT64_C(1)<<i;return mask;}
uint32_t sil_diagnostic(unsigned i,unsigned field){const auto&e=engine->core.entries[i];switch(field){case 0:return e.node;case 1:return e.bus_index;case 2:return e.last_motion_rx_tick_ms;case 3:return e.last_rx_tick_ms;case 4:return e.profile!=nullptr;case 5:return e.measured.has_motion_state;case 6:return e.protocol_error_count;case 7:return e.dev_init;default:return engine->core.entry_count;}}
void sil_disable(uint64_t mask){agent_gate_request_disable(&engine->gate,mask);}
void sil_snapshot(float*values,uint32_t*states,uint64_t*head){
 motor_control_motion_state_t motions[N];motor_control_status_t status[N];
 motor_control_copy_motion_states(&engine->core,motions,N,engine->tick);
 motor_control_copy_statuses(&engine->core,status,N,engine->tick);
 for(unsigned i=0;i<N;++i){
  values[i*3]=motions[i].position_rad;values[i*3+1]=motions[i].velocity_rad_s;values[i*3+2]=motions[i].effort_nm;
  states[i*10]=status[i].online;states[i*10+1]=status[i].policy_enabled;states[i*10+2]=status[i].motion_enabled;
  states[i*10+3]=status[i].control_lifecycle;states[i*10+4]=status[i].execution_state;
  states[i*10+5]=motions[i].state_age_ms;states[i*10+6]=status[i].policy_generation;
  states[i*10+7]=status[i].unified_fault_code;states[i*10+8]=motions[i].target_source;states[i*10+9]=engine->device[i].outputs;
 }
 const auto &a=engine->gate.authority;head[0]=a.lease_id;head[1]=a.sequence;head[2]=a.valid_until_tick_ms;
 head[3]=control_lease_allows(&a,a.lease_id,engine->tick);head[4]=engine->gate.link.command_sequence;
 head[5]=engine->gate.link.barrier_generation;head[6]=engine->gate.link.stop_generation;head[7]=engine->tick;
}
}
