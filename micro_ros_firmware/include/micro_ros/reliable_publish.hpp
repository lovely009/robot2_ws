#ifndef RELIABLE_PUBLISH_HPP
#define RELIABLE_PUBLISH_HPP

// 仅 micro_ros_task 的 Status/Info 发布调用使用；成功表示序列化并入队，
// ACK 与重传仍由 XRCE 可靠历史持有，不用于连接、命令或配置执行确认。
class ReliableTelemetryQueueScope {
public:
  ReliableTelemetryQueueScope();
  ~ReliableTelemetryQueueScope();
  ReliableTelemetryQueueScope(const ReliableTelemetryQueueScope &) = delete;
  ReliableTelemetryQueueScope &operator=(const ReliableTelemetryQueueScope &) = delete;
private:
  bool previous_;
};

#endif
