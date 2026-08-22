// 固定容量电机控制内核：不依赖 HAL、RTOS、Arduino 或 ROS。

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONTROL_MAX_MOTORS 8U
#define MOTOR_CONTROL_MAX_BUSES 8U
#define MOTOR_CONTROL_MAX_TX_ACTIONS 16U
#define MOTOR_CONTROL_JOINT_NAME_CAPACITY 32U

typedef enum {
  MOTOR_CONTROL_RP,
  MOTOR_CONTROL_VAXOR,
} motor_control_protocol_t;

typedef enum {
  MOTOR_CONTROL_DISABLED,
  MOTOR_CONTROL_ACTIVE,
  MOTOR_CONTROL_SOFT_STOP,
  MOTOR_CONTROL_FAULT_LATCHED,
} motor_control_state_t;

typedef struct {
  const char *joint_name;
  motor_control_protocol_t protocol;
  uint8_t id;
  uint8_t bus_index;
  double gear_ratio;
  double gear_efficiency;
  double torque_constant_nm_per_amp;
  double default_kp;
  double default_kd;
} motor_control_motor_config_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  uint8_t mode;
  double position;
  double velocity;
  double effort;
} motor_control_command_t;

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  uint8_t size;
  bool is_fd;
  bool is_extended;
  uint8_t data[64];
} motor_control_tx_action_t;

typedef struct {
  motor_control_state_t state;
  motor_protocol_joint_command_t command;
  uint32_t last_command_ms;
  bool command_valid;
  bool fault_active;
  uint8_t requested_mode;
  uint8_t active_mode;
  bool vaxor_enabled;
  bool vaxor_stop_pending;
  bool reauthorization_required;
} motor_control_motor_runtime_t;

typedef struct motor_control_context {
  motor_control_motor_config_t motors[MOTOR_CONTROL_MAX_MOTORS];
  char joint_names[MOTOR_CONTROL_MAX_MOTORS]
                  [MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  motor_control_motor_runtime_t runtime[MOTOR_CONTROL_MAX_MOTORS];
  bool bus_fault[MOTOR_CONTROL_MAX_BUSES];
  bool rp_bus_disabled[MOTOR_CONTROL_MAX_BUSES];
  size_t motor_count;
  uint32_t valid_command_count;
  uint32_t invalid_command_count;
  bool authorized;
  bool agent_connected;
  bool agent_ever_connected;
} motor_control_context_t;

void motor_control_init(motor_control_context_t *ctx,
                        const motor_control_motor_config_t *motors,
                        size_t motor_count);
bool motor_control_accept_command(motor_control_context_t *ctx,
                                  const motor_control_command_t *command,
                                  uint32_t now_ms);
size_t motor_control_step(motor_control_context_t *ctx, uint32_t now_ms,
                          motor_control_tx_action_t *actions,
                          size_t action_capacity);
void motor_control_set_agent_connected(motor_control_context_t *ctx,
                                       bool connected);
void motor_control_set_bus_fault(motor_control_context_t *ctx,
                                 uint8_t bus_index, bool active);
void motor_control_set_motor_fault(motor_control_context_t *ctx,
                                   size_t motor_index, bool active);
bool motor_control_set_authorization(motor_control_context_t *ctx,
                                     bool authorized);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_H
