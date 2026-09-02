#include <rmw_microxrcedds_c/config.h>

// rclc 参数服务器会创建 get/get_types/set/list/describe 五个 service。
// 这里检查实际链接进固件的 libmicroros 配置，而不是只检查项目 meta 文件。
static_assert(RMW_UXRCE_MAX_SERVICES >= 5,
              "micro-ROS 参数服务器需要至少 5 个 service 槽位");

int main() { return 0; }
