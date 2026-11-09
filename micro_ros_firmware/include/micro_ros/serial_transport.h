#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H
#include <Arduino.h>
void serial_transport_setup(Stream &serial);
// 仅 micro_ros 任务使用；一次高级发送的所有 framing 回调共享 2 ms 写预算。
void serial_transport_send_begin(void);
void serial_transport_send_end(void);
#endif
