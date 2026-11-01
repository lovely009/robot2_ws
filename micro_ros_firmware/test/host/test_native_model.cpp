#include <cassert>
#include <cmath>
#include <cstdint>
extern "C" {
int sil_init_model(unsigned,const char *const *,const uint8_t*);
void sil_advance(uint32_t);int sil_lease(uint64_t,uint32_t,uint32_t,int);
int sil_policy(uint64_t,uint32_t,uint32_t,uint32_t,unsigned,const uint16_t*,const uint8_t*,const float*,const float*);
int sil_command(uint64_t,uint32_t,uint32_t,unsigned,const uint8_t*,const int16_t*,const int16_t*,const int16_t*);
void sil_snapshot(float*,uint32_t*,uint64_t*);void sil_stop();uint64_t sil_enabled_mask();
}
int main(){
 const char *models[18];uint8_t buses[18]{},ids[18],enabled[18];uint16_t policy_ids[18];
 float kp[18],kd[18];int16_t p[18],v[18]{},t[18]{};
 for(unsigned i=0;i<18;++i){
  unsigned joint=i%9;
  models[i]=joint>=7?"vaxor_2550":joint==2||joint==3?"cerb_r58_101":"cerb_r48_101";
  ids[i]=policy_ids[i]=i+1;enabled[i]=1;kp[i]=joint>=7?.008f:30.f;kd[i]=1;p[i]=131;
 }
 assert(!sil_init_model(0,models,buses));
 const char *bad[]={"unknown"};assert(!sil_init_model(1,bad,buses));
 uint8_t invalid_bus[]={3};assert(!sil_init_model(1,models,invalid_bus));
 assert(sil_init_model(18,models,buses));
 sil_advance(1001);
 assert(sil_command(0,1,1030,0,ids,p,v,t));assert(sil_lease(100000001,1,1200,1));sil_advance(1005);
 assert(sil_policy(100000001,1,1,1105,18,policy_ids,enabled,kp,kd));
 unsigned seq=1,lease_seq=1;
 for(unsigned tick=1006;tick<=1600;++tick){
  sil_advance(tick);
  if(tick%100==0)assert(sil_lease(100000001,++lease_seq,tick+200,1));
  if(tick%20==0)assert(sil_command(100000001,++seq,tick+30,18,ids,p,v,t));
 }
 float values[55]{};uint32_t states[181]{};uint64_t head[8];
 values[54]=999;states[180]=999;sil_snapshot(values,states,head);
 assert(values[54]==999&&states[180]==999);
 for(unsigned i=0;i<18;++i){assert(states[i*10+1]);assert(states[i*10+9]>0);assert(std::abs(values[i*3]-.05f)<.01f);}
 sil_stop();sil_advance(1720);assert(sil_enabled_mask()==0);
}
