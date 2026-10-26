#include "robot_transmission/model.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <transmission_interface/transmission_loader.hpp>
#include <transmission_interface/handle.hpp>

namespace robot_transmission {
namespace {
std::vector<double> numbers(const YAML::Node &node, const char *key, std::size_t size, double fallback, bool ratio) {
  std::vector<double> result(size, fallback);
  if (node[key]) result = node[key].as<std::vector<double>>();
  if (result.size() != size) throw std::invalid_argument(std::string(key) + " length mismatch");
  for (double n : result) {
    if (!std::isfinite(n) || (ratio && n == 0.0)) throw std::invalid_argument(std::string(key) + " must be finite and reductions nonzero");
  }
  return result;
}
struct RuntimeGroup {
  Group config;
  std::shared_ptr<transmission_interface::TransmissionLoader> factory;
  std::shared_ptr<transmission_interface::Transmission> transmission;
  std::vector<Axis> actuators;
  std::vector<Axis> joints;
  void configure() {
    actuators.resize(config.actuators.size());
    joints.resize(config.joints.size());
    std::vector<transmission_interface::ActuatorHandle> a;
    std::vector<transmission_interface::JointHandle> j;
    // ROS get_names() 会排序；内部位置编号固定 YAML 定义的轴顺序。
    for (std::size_t i=0; i<actuators.size(); ++i) {
      const auto name="actuator_"+std::to_string(1000000+i);
      a.emplace_back(name,"position",&actuators[i].position);
      a.emplace_back(name,"velocity",&actuators[i].velocity);
      a.emplace_back(name,"effort",&actuators[i].effort);
    }
    for (std::size_t i=0; i<joints.size(); ++i) {
      const auto name="joint_"+std::to_string(1000000+i);
      j.emplace_back(name,"position",&joints[i].position);
      j.emplace_back(name,"velocity",&joints[i].velocity);
      j.emplace_back(name,"effort",&joints[i].effort);
    }
    transmission->configure(j,a);
  }
};
bool finite(const Axis &a) { return std::isfinite(a.position) && std::isfinite(a.velocity) && std::isfinite(a.effort); }
}
struct Model::Impl {
  pluginlib::ClassLoader<transmission_interface::TransmissionLoader> loader{"transmission_interface", "transmission_interface::TransmissionLoader"};
  std::vector<std::string> actuator_names;
  std::vector<std::string> joint_names;
  std::map<std::string,int> actuator_ids;
  std::vector<Group> groups;
  std::vector<std::unique_ptr<RuntimeGroup>> runtime;
  void add(Group g) {
    if (g.actuators.empty() || g.joints.empty() || g.name.empty()) throw std::invalid_argument("empty transmission group");
    hardware_interface::TransmissionInfo info;
    info.name=g.name; info.type=g.plugin;
    info.parameters.insert(g.parameters.begin(),g.parameters.end());
    for (std::size_t i=0;i<g.actuators.size();++i) {
      hardware_interface::ActuatorInfo a;
      a.name="actuator_"+std::to_string(1000000+i); a.role="actuator"+std::to_string(i+1);
      a.mechanical_reduction=g.actuator_reduction[i]; info.actuators.push_back(a);
    }
    for (std::size_t i=0;i<g.joints.size();++i) {
      hardware_interface::JointInfo j;
      j.name="joint_"+std::to_string(1000000+i); j.role="joint"+std::to_string(i+1);
      j.mechanical_reduction=g.joint_reduction[i];
      // SimpleTransmissionLoader 从 joint 读取单轴的总传动比。
      if (g.plugin=="transmission_interface/SimpleTransmission" && g.actuators.size()==1) {
        j.mechanical_reduction*=g.actuator_reduction[0];
        if (!std::isfinite(j.mechanical_reduction) || j.mechanical_reduction==0.0) {
          throw std::invalid_argument("invalid combined reduction for "+g.name);
        }
      }
      j.offset=g.joint_offset[i]; info.joints.push_back(j);
    }
    auto runtime_group=std::make_unique<RuntimeGroup>();
    runtime_group->config=g;
    // 工厂必须比插件实例活得久，第三方插件的 vtable 可能位于动态库。
    runtime_group->factory=loader.createSharedInstance(g.plugin);
    runtime_group->transmission=runtime_group->factory->load(info);
    if (!runtime_group->transmission) throw std::invalid_argument("transmission plugin rejected group "+g.name);
    runtime_group->configure();
    groups.push_back(g);
    joint_names.insert(joint_names.end(),g.joints.begin(),g.joints.end());
    runtime.push_back(std::move(runtime_group));
  }
  AxisMap transform(const AxisMap &input, bool to_joint) {
    const auto &known=to_joint ? actuator_names : joint_names;
    for (const auto &[name,sample] : input) {
      if (std::find(known.begin(),known.end(),name)==known.end()) throw std::invalid_argument("unknown axis "+name);
      if (!finite(sample)) throw std::invalid_argument("nonfinite sample "+name);
    }
    AxisMap output;
    for (auto &entry : runtime) {
      auto &r=*entry;
      const auto &src_names=to_joint ? r.config.actuators : r.config.joints;
      const auto &dst_names=to_joint ? r.config.joints : r.config.actuators;
      auto &src=to_joint ? r.actuators : r.joints;
      auto &dst=to_joint ? r.joints : r.actuators;
      std::size_t present=0;
      for (const auto &name : src_names) present+=input.count(name);
      if (present==0) continue;
      if (present != src_names.size()) {
        if (to_joint) continue;
        throw std::invalid_argument("incomplete command for transmission group "+r.config.name);
      }
      for (std::size_t i=0;i<src.size();++i) src[i]=input.at(src_names[i]);
      std::fill(dst.begin(),dst.end(),Axis{});
      if (to_joint) r.transmission->actuator_to_joint();
      else r.transmission->joint_to_actuator();
      for (std::size_t i=0;i<dst.size();++i) {
        if (!finite(dst[i])) throw std::invalid_argument("nonfinite transmission output "+dst_names[i]);
        output.emplace(dst_names[i],dst[i]);
      }
    }
    return output;
  }
};
Model::Model(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Model::~Model()=default;
Model::Model(Model &&) noexcept=default;
Model &Model::operator=(Model &&) noexcept=default;
Model Model::load(const std::string &driver_file, const std::string &transmissions_file) {
  namespace fs=std::filesystem;
  auto impl=std::make_unique<Impl>();
  auto driver=YAML::LoadFile(driver_file)["motor_hardware"];
  auto motors=driver["motors"];
  if (!motors.IsMap() || motors.size()==0) throw std::invalid_argument("driver requires motor_hardware.motors");
  std::map<std::string,std::string> legacy;
  for (const auto &entry : motors) {
    auto name=entry.first.as<std::string>();
    if (name.empty() || !entry.second["id"]) throw std::invalid_argument("motor needs name/id");
    const auto id=entry.second["id"].as<int>();
    if (id<0 || id>65535) throw std::invalid_argument("motor id outside uint16 range: "+name);
    if (!impl->actuator_ids.emplace(name,id).second) throw std::invalid_argument("duplicate motor "+name);
    impl->actuator_names.push_back(name);
    if (entry.second["joint"]) legacy[name]=entry.second["joint"].as<std::string>();
  }
  fs::path config;
  if (!transmissions_file.empty()) config=fs::path(driver_file).parent_path()/transmissions_file;
  else if (driver["transmissions_file"]) config=fs::path(driver_file).parent_path()/driver["transmissions_file"].as<std::string>();
  else if (fs::exists(fs::path(driver_file).parent_path()/"transmissions.yaml")) config=fs::path(driver_file).parent_path()/"transmissions.yaml";
  std::set<std::string> used_actuators,used_joints,used_groups;
  auto register_group=[&](Group g) {
    if (!used_groups.insert(g.name).second) throw std::invalid_argument("duplicate transmission name "+g.name);
    for (const auto &a:g.actuators) {
      if (!impl->actuator_ids.count(a) || !used_actuators.insert(a).second) throw std::invalid_argument("unknown or multiply owned actuator "+a);
    }
    for (const auto &j:g.joints) {
      if (j.empty() || !used_joints.insert(j).second) throw std::invalid_argument("empty or multiply owned joint "+j);
    }
    impl->add(std::move(g));
  };
  if (!config.empty()) {
    auto document=YAML::LoadFile(config.string());
    auto groups=document["transmissions"];
    if (!groups.IsMap()) throw std::invalid_argument("transmissions must be a map");
    for (const auto &entry:groups) {
      Group g;
      auto node=entry.second;
      const std::set<std::string> fields={"plugin","actuators","joints","actuator_reduction","joint_reduction","joint_offset","parameters"};
      for (const auto &field:node) {
        if (!fields.count(field.first.as<std::string>())) throw std::invalid_argument("unknown transmission field "+field.first.as<std::string>());
      }
      g.name=entry.first.as<std::string>(); g.plugin=node["plugin"].as<std::string>();
      g.actuators=node["actuators"].as<std::vector<std::string>>();
      g.joints=node["joints"].as<std::vector<std::string>>();
      g.actuator_reduction=numbers(node,"actuator_reduction",g.actuators.size(),1,true);
      g.joint_reduction=numbers(node,"joint_reduction",g.joints.size(),1,true);
      g.joint_offset=numbers(node,"joint_offset",g.joints.size(),0,false);
      if (node["parameters"]) g.parameters=node["parameters"].as<std::map<std::string,std::string>>();
      register_group(std::move(g));
    }
  }
  for (const auto &a:impl->actuator_names) {
    if (used_actuators.count(a)) continue;
    auto it=legacy.find(a);
    if (it==legacy.end() || it->second.empty()) throw std::invalid_argument("unmapped actuator "+a);
    register_group(Group{"direct_"+a,"transmission_interface/SimpleTransmission",{a},{it->second},{1},{1},{0},{}});
  }
  return Model(std::move(impl));
}
const std::vector<std::string> &Model::actuator_names() const { return impl_->actuator_names; }
const std::vector<std::string> &Model::joint_names() const { return impl_->joint_names; }
const std::map<std::string,int> &Model::actuator_ids() const { return impl_->actuator_ids; }
const std::vector<Group> &Model::groups() const { return impl_->groups; }
AxisMap Model::actuator_to_joint(const AxisMap &state) const { return impl_->transform(state,true); }
AxisMap Model::joint_to_actuator(const AxisMap &state) const { return impl_->transform(state,false); }
std::map<std::string,double> Model::joint_effort_to_actuator(const std::map<std::string,double> &effort) const {
  AxisMap input;
  for (const auto &[name,value]:effort) input.emplace(name,Axis(0,0,value));
  std::map<std::string,double> output;
  for (const auto &[name,axis]:joint_to_actuator(input)) output.emplace(name,axis.effort);
  return output;
}
std::map<std::string,double> Model::actuator_effort_to_joint(const std::map<std::string,double> &effort) const {
  AxisMap input;
  for (const auto &[name,value]:effort) input.emplace(name,Axis(0,0,value));
  std::map<std::string,double> output;
  for (const auto &[name,axis]:actuator_to_joint(input)) output.emplace(name,axis.effort);
  return output;
}
std::vector<std::string> Model::actuators_for_joint(const std::string &joint) const {
  for (const auto &g:impl_->groups) if (std::find(g.joints.begin(),g.joints.end(),joint)!=g.joints.end()) return g.actuators;
  throw std::invalid_argument("unknown joint "+joint);
}
}  // namespace robot_transmission
