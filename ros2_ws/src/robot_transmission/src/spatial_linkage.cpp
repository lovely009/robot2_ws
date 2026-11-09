#include "robot_transmission/spatial_linkage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot_transmission {
namespace {
using Vec = std::array<double,3>;
const double pi = std::acos(-1.0);
constexpr double tolerance = 1e-10;
Vec add(Vec a, Vec b) { return {a[0]+b[0],a[1]+b[1],a[2]+b[2]}; }
Vec sub(Vec a, Vec b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
Vec mul(Vec a, double s) { return {a[0]*s,a[1]*s,a[2]*s}; }
double dot(Vec a, Vec b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Vec cross(Vec a, Vec b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
Vec rotate(Vec axis, double angle, Vec v) {
  return add(add(mul(v,std::cos(angle)),mul(cross(axis,v),std::sin(angle))),
             mul(axis,dot(axis,v)*(1-std::cos(angle))));
}
void require(bool ok, const std::string &reason) {
  if (!ok) throw std::invalid_argument("spatial linkage: "+reason);
}
double finite(double value) { require(std::isfinite(value),"nonfinite calculation"); return value; }
std::vector<double> parse(const std::string &text, std::size_t count) {
  std::istringstream stream(text);
  std::vector<double> result(count);
  for (double &v:result) require(bool(stream>>v) && std::isfinite(v),"invalid geometry parameter");
  stream>>std::ws;
  require(stream.eof(),"trailing geometry parameter data");
  return result;
}
// 同一旋转圆约束只保留零位导数同号的根；输出固定在 (-pi, pi]，不缓存上次解。
double solve(Vec center, Vec axis, Vec offset, Vec fixed, double length, double scale, double sign) {
  Vec parallel=mul(axis,dot(axis,offset));
  Vec v=sub(offset,parallel), w=cross(axis,v), d=sub(add(center,parallel),fixed);
  double a=finite(2*dot(d,v)/scale), b=finite(2*dot(d,w)/scale);
  double c=finite((dot(d,d)+dot(v,v)-length*length)/scale);
  double radius=std::hypot(a,b);
  require(radius>tolerance,"degenerate rotation circle");
  double cosine=finite(-c/radius);
  require(std::abs(cosine)<=1+64*std::numeric_limits<double>::epsilon(),"unreachable pose");
  cosine=std::clamp(cosine,-1.0,1.0);
  // acos 近重根会丢失逆解及雅可比精度；限制舍入误差放大，不充当机械限位。
  require((1-cosine*cosine)*tolerance>64*std::numeric_limits<double>::epsilon(),
          "numerically unresolved tangent branch");
  double alpha=std::acos(cosine);
  double delta=std::atan2(b,a);
  double answer=0;
  int count=0;
  for(double angle:{delta+alpha,delta-alpha}) {
    angle=std::remainder(angle,2*pi);
    double derivative=-a*std::sin(angle)+b*std::cos(angle);
    if(derivative*sign>tolerance) { answer=angle; ++count; }
  }
  require(count==1,"singular or ambiguous branch");
  return answer;
}
template<class Handle>
std::vector<Handle> bind_handles(
  const std::vector<Handle> &handles, const std::array<std::string,2> &names)
{
  require(handles.size()==6,"requires two complete P/V/E handle sets");
  std::vector<Handle> result;
  result.reserve(6);
  const std::array<std::string,3> interfaces{"position","velocity","effort"};
  for(std::size_t i=0;i<2;++i) for(std::size_t k=0;k<3;++k) {
    std::size_t count=0;
    for(const auto &h:handles) if(h.get_name()==names[i]+"/"+interfaces[k]) {
      require(bool(h),"null handle"); result.push_back(h); ++count;
    }
    require(count==1,"missing or duplicate handle");
  }
  return result;
}
}
SpatialLinkageTransmission::SpatialLinkageTransmission(const hardware_interface::TransmissionInfo &info) {
  require(info.actuators.size()==2 && info.joints.size()==2,"requires two actuators and joints");
  for(std::size_t i=0;i<2;++i) {
    signs_[i]=info.actuators[i].mechanical_reduction;
    require(signs_[i]==1 || signs_[i]==-1,"actuator reduction must be +1 or -1");
    require(info.joints[i].mechanical_reduction==1 && info.joints[i].offset==0,"joint reduction/offset unsupported");
    actuator_names_[i]=info.actuators[i].name; joint_names_[i]=info.joints[i].name;
  }
  require(!actuator_names_[0].empty() && !actuator_names_[1].empty() && actuator_names_[0]!=actuator_names_[1] &&
          !joint_names_[0].empty() && !joint_names_[1].empty() && joint_names_[0]!=joint_names_[1],"invalid axis names");
  const std::set<std::string> keys{
    "motor_axis_origin","motor_axis_direction","motor_ball_zero","joint1_axis_origin",
    "joint1_axis_direction","joint2_axis_origin","joint2_axis_direction","joint_ball_zero","rod_length"};
  require(info.parameters.size()==keys.size(),"missing or unknown geometry parameter");
  for(const auto &[key,value]:info.parameters) { (void)value; require(keys.count(key),"unknown parameter "+key); }
  auto vector=[&](const std::string &key) {
    auto v=parse(info.parameters.at(key),3); return Vec{v[0],v[1],v[2]};
  };
  motor_origin_=vector("motor_axis_origin"); motor_axis_=vector("motor_axis_direction");
  motor_ball_=vector("motor_ball_zero"); origin1_=vector("joint1_axis_origin");
  axis1_=vector("joint1_axis_direction"); origin2_=vector("joint2_axis_origin");
  axis2_=vector("joint2_axis_direction"); joint_ball_=vector("joint_ball_zero");
  length_=parse(info.parameters.at("rod_length"),1)[0];
  require(length_>0,"rod length must be positive");
  for(auto axis:{motor_axis_,axis1_,axis2_}) require(std::abs(dot(axis,axis)-1)<=tolerance,"axis must be unit length");
  scale_=finite(length_*length_);
  require(scale_>0,"rod length underflow");
  require(dot(cross(motor_axis_,sub(motor_ball_,motor_origin_)),cross(motor_axis_,sub(motor_ball_,motor_origin_)))/scale_>tolerance &&
          dot(cross(axis2_,sub(joint_ball_,origin2_)),cross(axis2_,sub(joint_ball_,origin2_)))/scale_>tolerance,"ball lies on rotation axis");
  phase_=std::atan2(motor_ball_[1]-motor_origin_[1],-(motor_ball_[2]-motor_origin_[2]));
  check_motor_branch(0);
  auto d=derivatives(0,0,0);
  pitch_derivative_sign_=std::copysign(1.,d[1]); motor_derivative_sign_=std::copysign(1.,d[2]);
  require(std::abs(d[1])>tolerance && std::abs(d[2])>tolerance,"singular zero pose");
  forward(0,0);
}
void SpatialLinkageTransmission::configure(
  const std::vector<transmission_interface::JointHandle> &joints,
  const std::vector<transmission_interface::ActuatorHandle> &actuators)
{
  configured_=false;
  auto j=bind_handles(joints,joint_names_); auto a=bind_handles(actuators,actuator_names_);
  joints_=std::move(j); actuators_=std::move(a); configured_=true;
}
SpatialLinkageTransmission::Vec SpatialLinkageTransmission::motor_ball(double t) const {
  return add(motor_origin_,rotate(motor_axis_,t,sub(motor_ball_,motor_origin_)));
}
SpatialLinkageTransmission::Vec SpatialLinkageTransmission::joint_ball(double q1, double q2) const {
  return add(origin1_,rotate(axis1_,q1,add(sub(origin2_,origin1_),rotate(axis2_,q2,sub(joint_ball_,origin2_)))));
}
void SpatialLinkageTransmission::check_motor_branch(double t) const {
  require(std::isfinite(t) && phase_+t>0 && phase_+t<pi,"outside input half-circle branch");
}
std::array<double,3> SpatialLinkageTransmission::derivatives(double q1, double q2, double t) const {
  Vec a=motor_ball(t), b=joint_ball(q1,q2), d=sub(b,a);
  require(std::abs(finite(dot(d,d)/scale_)-1)<=tolerance,"rod does not close");
  return {finite(2*dot(d,cross(axis1_,sub(b,origin1_)))/scale_),
          finite(2*dot(d,rotate(axis1_,q1,cross(axis2_,rotate(axis2_,q2,sub(joint_ball_,origin2_)))))/scale_),
          finite(-2*dot(d,cross(motor_axis_,sub(a,motor_origin_)))/scale_)};
}
double SpatialLinkageTransmission::forward(double q1, double t) const {
  check_motor_branch(t);
  // 将上球心旋转圆整体变换到 base，再解 q2。
  double q2=solve(add(origin1_,rotate(axis1_,q1,sub(origin2_,origin1_))),rotate(axis1_,q1,axis2_),
                  rotate(axis1_,q1,sub(joint_ball_,origin2_)),motor_ball(t),length_,scale_,pitch_derivative_sign_);
  auto d=derivatives(q1,q2,t);
  require(d[1]*pitch_derivative_sign_>tolerance && d[2]*motor_derivative_sign_>tolerance,"singular or opposite branch");
  // 正向也校验输入圆反解，避免输出角已舍入到无法恢复电机姿态的切点。
  const double recovered=solve(motor_origin_,motor_axis_,sub(motor_ball_,motor_origin_),joint_ball(q1,q2),
                               length_,scale_,motor_derivative_sign_);
  require(std::abs(recovered-t)<=tolerance,"motor pose is not numerically reversible");
  return q2;
}
double SpatialLinkageTransmission::inverse(double q1, double q2) const {
  double t=solve(motor_origin_,motor_axis_,sub(motor_ball_,motor_origin_),joint_ball(q1,q2),
                 length_,scale_,motor_derivative_sign_);
  require(std::abs(forward(q1,t)-q2)<=tolerance,"inverse does not match output branch");
  return t;
}
void SpatialLinkageTransmission::convert(bool to_joint) {
  require(configured_,"handles not configured");
  std::array<Axis,2> input, output;
  for(std::size_t i=0;i<2;++i) {
    for(std::size_t k=0;k<3;++k) finite(to_joint?actuators_[3*i+k].get_value():joints_[3*i+k].get_value());
    input[i]=to_joint?Axis(actuators_[3*i].get_value(),actuators_[3*i+1].get_value(),actuators_[3*i+2].get_value()):
                     Axis(joints_[3*i].get_value(),joints_[3*i+1].get_value(),joints_[3*i+2].get_value());
  }
  double q1=to_joint?signs_[0]*input[0].position:input[0].position;
  double t=to_joint?signs_[1]*input[1].position:inverse(q1,input[1].position);
  double q2=to_joint?forward(q1,t):input[1].position;
  auto d=derivatives(q1,q2,t);
  double j21=finite(-d[0]/d[1]*signs_[0]), j22=finite(-d[2]/d[1]*signs_[1]);
  require(std::abs(j22)>tolerance,"singular Jacobian");
  if(to_joint) {
    output[1]=Axis(q2,j21*input[0].velocity+j22*input[1].velocity,input[1].effort/j22);
    output[0]=Axis(q1,signs_[0]*input[0].velocity,(input[0].effort-j21*output[1].effort)/signs_[0]);
  } else {
    output[0]=Axis(q1/signs_[0],input[0].velocity/signs_[0],signs_[0]*input[0].effort+j21*input[1].effort);
    output[1]=Axis(t/signs_[1],(input[1].velocity-j21*output[0].velocity)/j22,j22*input[1].effort);
  }
  for(const auto &axis:output) { finite(axis.position); finite(axis.velocity); finite(axis.effort); }
  // 整组先验证再写入，任何异常都不得留下半组输出。
  for(std::size_t i=0;i<2;++i) {
    const std::array<double,3> values{output[i].position,output[i].velocity,output[i].effort};
    for(std::size_t k=0;k<3;++k) {
      if(to_joint) joints_[3*i+k].set_value(values[k]); else actuators_[3*i+k].set_value(values[k]);
    }
  }
}
void SpatialLinkageTransmission::actuator_to_joint() { convert(true); }
void SpatialLinkageTransmission::joint_to_actuator() { convert(false); }
std::shared_ptr<transmission_interface::Transmission> SpatialLinkageTransmissionLoader::load(
  const hardware_interface::TransmissionInfo &info)
{
  return std::make_shared<SpatialLinkageTransmission>(info);
}
}  // namespace robot_transmission
