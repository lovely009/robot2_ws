#include "robot_transmission/spatial_linkage.hpp"
#include <pluginlib/class_list_macros.hpp>

// 注册必须位于独立插件库，避免核心库作为硬件依赖加载时归属外层 ClassLoader。
PLUGINLIB_EXPORT_CLASS(robot_transmission::SpatialLinkageTransmissionLoader,
                      transmission_interface::TransmissionLoader)
