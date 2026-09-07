// newlib-nano syscall 兜底：libmicroros（rcutils 日志初始化错误路径的 fprintf、
// rosidl string 函数的 fwrite）把 newlib stdio 写路径链入 libc_nano.a(writer.o)，
// 其中 _write 是强引用；框架的 _write 弱定义在 Print.cpp.o（cores/arduino/api），
// 固件代码不使用 Print/printf，该目标文件永远不会被链接器提取，
// 导致 "undefined reference to `_write'"。
// 此处提供强定义空汇：MCU 上禁止动态日志/printf（不接 USB CDC，避免与
// micro-ROS serial 传输抢占带宽与确定性）；新增 stdio 用户须重新审视本文件。
extern "C" int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}
