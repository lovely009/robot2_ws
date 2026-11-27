# 固件原生服务 /motor/get_device_info 返回的设备拓扑与参数快照（Schema 6）。
uint16 SCHEMA_VERSION=6
std_msgs/Header header
uint16 schema_version
uint64 session_id
uint32 info_revision
MotorTopologyEntry[<=37] topology
DeviceParamEntry[<=37] params
