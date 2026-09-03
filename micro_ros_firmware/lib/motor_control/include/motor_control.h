// 固定容量电机控制内核：支持 vaxor 25/30mm 与 CREB CAN FD/MIT。

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
  MOTOR_CONTROL_VAXOR,
  MOTOR_CONTROL_CREB,
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
  double mit_position_limit_rad;
  double mit_velocity_limit_rad_s;
  double mit_torque_limit_nm;
} motor_control_motor_config_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  double position;
  double velocity;
  double effort;
} motor_control_command_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  uint8_t mode;
} motor_control_mode_request_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  double kp;
  double kd;
} motor_control_gains_request_t;

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  uint8_t size;
  bool is_fd;
  bool is_extended;
  uint8_t data[64];
} motor_control_tx_action_t;

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  uint8_t size;
  bool is_fd;
  bool is_extended;
  uint8_t data[64];
} motor_control_rx_frame_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  double position;
  double velocity;
  double effort;
} motor_control_joint_state_t;

typedef struct {
  char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  uint32_t sequence;
  double voltage_v;
  uint32_t fault;
  uint8_t motor_state;
  uint8_t calibration_state;
  uint8_t mode;
  uint8_t motor_type;
} motor_control_motor_status_t;

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
  bool vaxor_mode_change_pending;
  bool creb_enabled;
  bool creb_disable_pending;
  uint32_t creb_enable_started_ms;
  double kp;
  double kd;
  bool reauthorization_required;
} motor_control_motor_runtime_t;

typedef struct motor_control_context {
  motor_control_motor_config_t motors[MOTOR_CONTROL_MAX_MOTORS];
  char joint_names[MOTOR_CONTROL_MAX_MOTORS]
                  [MOTOR_CONTROL_JOINT_NAME_CAPACITY];
  motor_control_motor_runtime_t runtime[MOTOR_CONTROL_MAX_MOTORS];
  motor_control_joint_state_t joint_states[MOTOR_CONTROL_MAX_MOTORS];
  bool joint_state_valid[MOTOR_CONTROL_MAX_MOTORS];
  motor_control_motor_status_t motor_statuses[MOTOR_CONTROL_MAX_MOTORS];
  bool motor_status_valid[MOTOR_CONTROL_MAX_MOTORS];
  bool bus_fault[MOTOR_CONTROL_MAX_BUSES];
  size_t motor_count;
  uint32_t valid_command_count;
  uint32_t invalid_command_count;
  uint32_t valid_feedback_count;
  uint32_t invalid_feedback_count;
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
bool motor_control_accept_feedback(motor_control_context_t *ctx,
                                   const motor_control_rx_frame_t *frame);
bool motor_control_get_joint_state(const motor_control_context_t *ctx,
                                   size_t index,
                                   motor_control_joint_state_t *out);
bool motor_control_get_motor_status(const motor_control_context_t *ctx,
                                    size_t index,
                                    motor_control_motor_status_t *out);
bool motor_control_set_mode(motor_control_context_t *ctx,
                            const char *joint_name, uint8_t mode);
bool motor_control_set_gains(motor_control_context_t *ctx,
                             const char *joint_name, double kp, double kd);
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
