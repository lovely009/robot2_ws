// Exact-boundary replacement and stale-source regression, with fictional axes.
#include <cassert>
#include <cstdint>
#include <cstdio>
extern "C" {
void sil_init(); void sil_advance(uint32_t);
int sil_lease(uint64_t,uint32_t,uint32_t,int);
int sil_command(uint64_t,uint32_t,uint32_t,unsigned,const uint8_t*,const int16_t*,const int16_t*,const int16_t*);
int sil_policy(uint64_t,uint32_t,uint32_t,uint32_t,unsigned,const uint16_t*,const uint8_t*,const float*,const float*);
void sil_snapshot(float*,uint32_t*,uint64_t*); uint64_t sil_enabled_mask();
}
int main(){
  sil_init();sil_advance(1020);
  constexpr uint64_t lease=100000001;
  uint8_t ids[37],enabled[37];uint16_t policy_ids[37];int16_t p[37]{},v[37]{},t[37]{};float kp[37],kd[37];
  for(unsigned i=0;i<37;++i){ids[i]=policy_ids[i]=i+1;enabled[i]=1;kp[i]=30;kd[i]=1;}
  assert(sil_command(0,1,1050,0,ids,p,v,t));
  assert(sil_lease(lease,1,1220,1));sil_advance(1025);
  assert(sil_policy(lease,7,1,1120,37,policy_ids,enabled,kp,kd));sil_advance(1030);
  sil_advance(1044);assert(sil_command(lease,2,1070,37,ids,p,v,t));sil_advance(1045);
  // A fresh replacement is already queued for the old deadline's boundary.
  sil_advance(1069);assert(sil_command(lease,3,1090,37,ids,p,v,t));sil_advance(1070);
  float values[111];uint32_t states[370];uint64_t head[8];
  sil_snapshot(values,states,head);
  for(unsigned i=0;i<37;++i){assert(states[i*10+1]);assert(states[i*10+4]!=5);}
  // Without a new target, the ORIGINAL deadline still stops every axis.
  sil_advance(1090);sil_snapshot(values,states,head);
  for(unsigned i=0;i<37;++i)assert(states[i*10+4]==5);
  // Ownership renewals alone cannot revive the stale target source.
  assert(sil_lease(lease,2,1290,1));sil_advance(1175);
  sil_snapshot(values,states,head);assert(!head[3]);assert(sil_enabled_mask()==0);
  for(unsigned i=0;i<37;++i)assert(!states[i*10+1]);
  std::puts("37-axis exact-boundary replacement, original deadline and stale-source stop passed");
  for(unsigned phase=0;phase<20;++phase){
    sil_init();sil_advance(1001);
    assert(sil_command(0,1,1031,0,ids,p,v,t));
    assert(sil_lease(lease,1,1201,1));
    uint32_t command_seq=1,lease_seq=1,source=0,baseline[37]{};
    for(unsigned tick=1002;tick<=2300;++tick){
      sil_advance(tick);
      if(tick%20==phase)source=tick;
      if(tick%100==1)assert(sil_lease(lease,++lease_seq,tick+200,1));
      if(tick==1010)assert(sil_policy(lease,7,1,1110,37,policy_ids,enabled,kp,kd));
      if(source&&tick==source+8)assert(sil_command(lease,++command_seq,source+30,37,ids,p,v,t));
      if(tick==1300){sil_snapshot(values,states,head);for(unsigned i=0;i<37;++i)baseline[i]=states[i*10+9];}
    }
    sil_snapshot(values,states,head);
    for(unsigned i=0;i<37;++i){assert(states[i*10+1]);assert(states[i*10+9]-baseline[i]==200);}
  }
  std::puts("20 source phases / 8ms transport: every fictional axis exactly 200 outputs/s, 1ms admission, original 30ms deadline");
}
