// 为 newlib-nano 提供丢弃输出的 _write，满足 libmicroros 的 stdio 链接依赖。
// 禁止将日志接入 USB CDC，以免占用 micro-ROS 传输；新增 stdio 用户须复查。
extern "C" int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}
