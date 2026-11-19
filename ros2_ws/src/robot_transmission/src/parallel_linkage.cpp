#include "robot_transmission/parallel_linkage.hpp"
#include "robot_transmission/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot_transmission {
namespace {
using V=std::array<double,3>; using Pair=std::array<double,2>; using Matrix=std::array<Pair,2>;
constexpr double eps=1e-10;
V add(V a,V b) {return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
V sub(V a,V b) {return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
V mul(V a,double b) {return {a[0]*b,a[1]*b,a[2]*b};}
double dot(V a,V b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
V cross(V a,V b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
V rotate(V axis,double angle,V v) {
  return add(add(mul(v,std::cos(angle)),mul(cross(axis,v),std::sin(angle))),mul(axis,dot(axis,v)*(1-std::cos(angle))));
}
void require(bool ok,const std::string &text) {if(!ok) throw std::invalid_argument("parallel linkage: "+text);}
std::vector<double> parse(const std::string &text,std::size_t count) {
  std::istringstream stream(text); std::vector<double> values(count);
  for(auto &v:values) require(bool(stream>>v)&&std::isfinite(v),"invalid geometry");
  stream>>std::ws; require(stream.eof(),"trailing geometry data"); return values;
}
Matrix invert(Matrix a) {
  const double det=a[0][0]*a[1][1]-a[0][1]*a[1][0];
  const double scale=std::hypot(a[0][0],a[0][1])*std::hypot(a[1][0],a[1][1]);
  require(std::isfinite(det)&&std::isfinite(scale)&&scale>0&&std::abs(det)>eps*scale,"singular Jacobian");
  return {{{a[1][1]/det,-a[0][1]/det},{-a[1][0]/det,a[0][0]/det}}};
}
Pair product(Matrix a,Pair b) {return {a[0][0]*b[0]+a[0][1]*b[1],a[1][0]*b[0]+a[1][1]*b[1]};}
Matrix transpose(Matrix a) {return {{{a[0][0],a[1][0]},{a[0][1],a[1][1]}}};}
double norm(Pair a) {return std::hypot(a[0],a[1]);}
template<class Handle> std::vector<Handle> bind_handles(const std::vector<Handle> &handles,const std::array<std::string,2> &names) {
  require(handles.size()==6,"two complete P/V/E handle sets required"); std::vector<Handle> result;
  for(const auto &name:names) for(const auto *channel:{"position","velocity","effort"}) {
    std::size_t count=0;
    for(const auto &h:handles) if(h.get_name()==name+"/"+channel) {require(bool(h),"null handle");result.push_back(h);++count;}
    require(count==1,"missing or duplicate handle");
  }
  return result;
}
}
ParallelLinkageTransmission::ParallelLinkageTransmission(const hardware_interface::TransmissionInfo &info) {
  require(info.actuators.size()==2&&info.joints.size()==2,"two inputs and outputs required");
  std::set<std::string> used;
  auto read=[&](const std::string &key,std::size_t count) {
    used.insert(key); auto found=info.parameters.find(key);require(found!=info.parameters.end(),"missing "+key);return parse(found->second,count);
  };
  auto vec=[&](const std::string &key) {auto v=read(key,3);return V{v[0],v[1],v[2]};};
  const auto direct=read("direct_first",1)[0]; require(direct==0||direct==1,"invalid direct_first");direct_first_=direct==1;
  for(std::size_t i=0;i<2;++i) {
    signs_[i]=info.actuators[i].mechanical_reduction;
    require(signs_[i]==1||signs_[i]==-1,"input reduction must be +1 or -1");
    require(info.joints[i].mechanical_reduction==1&&info.joints[i].offset==0,"output reduction/offset unsupported");
    joint_names_[i]=info.joints[i].name;actuator_names_[i]=info.actuators[i].name;
    const auto prefix="joint"+std::to_string(i+1);
    origins_[i]=vec(prefix+"_origin");axes_[i]=vec(prefix+"_axis");
    require(std::abs(dot(axes_[i],axes_[i])-1)<1e-8,"output axis must be unit length");
    auto &r=rods_[i];const auto motor="motor"+std::to_string(i+1);
    const auto bounds=read(motor+"_bounds",2);r.lower=bounds[0];r.upper=bounds[1];
    require(r.lower<0&&r.upper>0,"input bounds must contain zero");
    if(i==0&&direct_first_) continue;
    r.origin=vec(motor+"_origin");r.axis=vec(motor+"_axis");r.ball=vec(motor+"_ball");r.platform=vec(motor+"_platform");
    r.length=read(motor+"_length",1)[0];require(r.length>1e-6&&std::abs(dot(r.axis,r.axis)-1)<1e-8,"invalid rod/axis");
  }
  // Both output frames must be initialized before evaluating either rod.
  for(std::size_t i=direct_first_?1:0;i<2;++i) {
    const auto d=constraint(i,{0,0},0);
    require(std::abs(d[0])<1e-8&&std::abs(d[3])>eps,"invalid or singular zero assembly");
    rods_[i].sign=std::copysign(1.,d[3]);
  }
  require(used.size()==info.parameters.size(),"unknown geometry parameter");
  require(!joint_names_[0].empty()&&!joint_names_[1].empty()&&joint_names_[0]!=joint_names_[1]&&
    !actuator_names_[0].empty()&&!actuator_names_[1].empty()&&actuator_names_[0]!=actuator_names_[1],"invalid axis names");
  invert(jacobian({0,0},{0,0}));
}
void ParallelLinkageTransmission::configure(const std::vector<transmission_interface::JointHandle> &joints,
  const std::vector<transmission_interface::ActuatorHandle> &actuators) {
  configured_=false;auto j=bind_handles(joints,joint_names_);auto a=bind_handles(actuators,actuator_names_);
  joints_=std::move(j);actuators_=std::move(a);configured_=true;
}
ParallelLinkageTransmission::V ParallelLinkageTransmission::platform(std::size_t i,Pair q) const {
  return add(origins_[0],rotate(axes_[0],q[0],add(sub(origins_[1],origins_[0]),rotate(axes_[1],q[1],sub(rods_[i].platform,origins_[1])))));
}
std::array<double,4> ParallelLinkageTransmission::constraint(std::size_t i,Pair q,double t) const {
  const auto &r=rods_[i];const auto a=add(r.origin,rotate(r.axis,t,sub(r.ball,r.origin))),b=platform(i,q),d=sub(b,a);
  const double scale=r.length*r.length;
  return {dot(d,d)/scale-1,2*dot(d,cross(axes_[0],sub(b,origins_[0])))/scale,
    2*dot(d,rotate(axes_[0],q[0],cross(axes_[1],rotate(axes_[1],q[1],sub(r.platform,origins_[1])))))/scale,
    -2*dot(d,cross(r.axis,sub(a,r.origin)))/scale};
}
ParallelLinkageTransmission::Pair ParallelLinkageTransmission::inverse(Pair q) const {
  Pair t{};
  for(std::size_t i=0;i<2;++i) {
    const auto &r=rods_[i];
    if(i==0&&direct_first_) {t[i]=q[0];require(t[i]>=r.lower-1e-9&&t[i]<=r.upper+1e-9,"direct axis outside input range");continue;}
    const auto radius=sub(r.ball,r.origin),parallel=mul(r.axis,dot(r.axis,radius)),radial=sub(radius,parallel);
    const auto centre=sub(sub(platform(i,q),r.origin),parallel);
    const double cosine=dot(centre,radial),sine=dot(centre,cross(r.axis,radial));
    const double offset=(dot(centre,centre)+dot(radial,radial)-r.length*r.length)/2;
    const double amplitude=std::hypot(cosine,sine);
    require(amplitude>1e-14,"degenerate motor circle");
    const double ratio=offset/amplitude;
    require(std::isfinite(ratio)&&std::abs(ratio)<=1+64*std::numeric_limits<double>::epsilon(),"unreachable output pose");
    require((1-ratio*ratio)*eps>64*std::numeric_limits<double>::epsilon(),"numerically unresolved tangent branch");
    const double delta=std::acos(std::clamp(ratio,-1.,1.)),phase=std::atan2(sine,cosine);
    std::size_t count=0;
    for(double angle:{phase+delta,phase-delta}) {
      angle=std::remainder(angle,2*std::acos(-1.));
      if(angle<r.lower-1e-9||angle>r.upper+1e-9) continue;
      const auto d=constraint(i,q,angle);
      if(d[3]*r.sign>eps) {t[i]=angle;++count;}
    }
    require(count==1,"unreachable or singular zero-assembly branch");
  }
  return t;
}
ParallelLinkageTransmission::Matrix ParallelLinkageTransmission::jacobian(Pair q,Pair t) const {
  Matrix a{};
  for(std::size_t i=0;i<2;++i) {
    if(i==0&&direct_first_) {a[0]={1/signs_[0],0};continue;}
    const auto d=constraint(i,q,t[i]);
    require(std::abs(d[0])<1e-8&&d[3]*rods_[i].sign>eps,"invalid rod closure or branch");
    a[i]={-d[1]/d[3]/signs_[i],-d[2]/d[3]/signs_[i]};
  }
  return a;
}
ParallelLinkageTransmission::Pair ParallelLinkageTransmission::forward(Pair target) const {
  for(std::size_t i=0;i<2;++i) require(target[i]>=rods_[i].lower-1e-9&&target[i]<=rods_[i].upper+1e-9,"input outside assembly range");
  // Deterministic continuation from the calibrated zero assembly. No retained
  // pose can select a different branch after a stop/restart or failed request.
  const int steps=std::max(1,static_cast<int>(std::ceil(std::max(std::abs(target[0]),std::abs(target[1]))/.1)));
  require(steps<=64,"input outside solver domain");Pair q{};
  for(int step=1;step<=steps;++step) {
    Pair t{target[0]*step/steps,target[1]*step/steps};
    auto residual=[&](Pair at) {return Pair{direct_first_?at[0]-t[0]:constraint(0,at,t[0])[0],constraint(1,at,t[1])[0]};};
    bool converged=false;
    for(int iteration=0;iteration<35;++iteration) {
      auto f=residual(q);if(norm(f)<1e-11) {converged=true;break;}
      auto d=constraint(1,q,t[1]);Matrix j{};j[1]={d[1],d[2]};
      if(direct_first_) j[0]={1,0};else {d=constraint(0,q,t[0]);j[0]={d[1],d[2]};}
      auto change=product(invert(j),f);double scale=std::min(1.,.2/std::max(norm(change),1e-15));bool accepted=false;
      for(int line=0;line<16;++line) {
        Pair next{q[0]-scale*change[0],q[1]-scale*change[1]};
        if(norm(residual(next))<norm(f)) {q=next;accepted=true;break;}scale*=.5;
      }
      require(accepted,"forward solve cannot converge");
    }
    require(converged,"forward solve iteration limit");
    const auto check=inverse(q);require(norm({check[0]-t[0],check[1]-t[1]})<1e-7,"forward pose is on another assembly branch");
  }
  return q;
}
void ParallelLinkageTransmission::convert(bool to_joint) {
  require(configured_,"handles not configured");std::array<Axis,2> input,output;
  for(std::size_t i=0;i<2;++i) {
    std::array<double,3> v{};
    for(std::size_t k=0;k<3;++k) {v[k]=to_joint?actuators_[3*i+k].get_value():joints_[3*i+k].get_value();require(std::isfinite(v[k]),"nonfinite input");}
    input[i]=Axis(v[0],v[1],v[2]);
  }
  Pair q{},t{};
  if(to_joint) {t={input[0].position*signs_[0],input[1].position*signs_[1]};q=forward(t);}
  else {q={input[0].position,input[1].position};t=inverse(q);const auto check=forward(t);require(norm({check[0]-q[0],check[1]-q[1]})<1e-7,"output is on another assembly branch");}
  const auto a=jacobian(q,t),inv=invert(a);
  Pair v{input[0].velocity,input[1].velocity},effort{input[0].effort,input[1].effort};
  const auto velocity=product(to_joint?inv:a,v),torque=product(transpose(to_joint?a:inv),effort);
  for(std::size_t i=0;i<2;++i) output[i]=Axis(to_joint?q[i]:t[i]/signs_[i],velocity[i],torque[i]);
  for(const auto &axis:output) require(std::isfinite(axis.position)&&std::isfinite(axis.velocity)&&std::isfinite(axis.effort),"nonfinite output");
  for(std::size_t i=0;i<2;++i) {
    const std::array<double,3> v{output[i].position,output[i].velocity,output[i].effort};
    for(std::size_t k=0;k<3;++k) {if(to_joint) joints_[3*i+k].set_value(v[k]);else actuators_[3*i+k].set_value(v[k]);}
  }
}
void ParallelLinkageTransmission::actuator_to_joint() {convert(true);}
void ParallelLinkageTransmission::joint_to_actuator() {convert(false);}
std::shared_ptr<transmission_interface::Transmission> ParallelLinkageTransmissionLoader::load(const hardware_interface::TransmissionInfo &info) {
  return std::make_shared<ParallelLinkageTransmission>(info);
}
}
