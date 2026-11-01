#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

// 注册容量与周期默认值；运行期周期由 firmware_settings 管理，线协议和 Flash 上界不变。
#define FIRMWARE_MAX_MOTORS 48U
#define FIRMWARE_MOTOR_REPORT_HZ 50U
#define FIRMWARE_MOTOR_COMMAND_HZ 200U

#if FIRMWARE_MAX_MOTORS == 0U || FIRMWARE_MAX_MOTORS > 64U
#error "FIRMWARE_MAX_MOTORS must be in [1, 64]"
#endif
#if FIRMWARE_MOTOR_REPORT_HZ == 0U || FIRMWARE_MOTOR_REPORT_HZ > 1000U
#error "FIRMWARE_MOTOR_REPORT_HZ must be in [1, 1000]"
#elif 1000U % FIRMWARE_MOTOR_REPORT_HZ != 0U
#error "Motor report period must be an integer number of milliseconds"
#endif
#if FIRMWARE_MOTOR_COMMAND_HZ == 0U || FIRMWARE_MOTOR_COMMAND_HZ > 1000U
#error "FIRMWARE_MOTOR_COMMAND_HZ must be in [1, 1000]"
#elif 1000U % FIRMWARE_MOTOR_COMMAND_HZ != 0U
#error "Motor command period must be an integer number of milliseconds"
#endif

#define FIRMWARE_MOTOR_REPORT_PERIOD_MS (1000U / FIRMWARE_MOTOR_REPORT_HZ)
#define FIRMWARE_MOTOR_COMMAND_PERIOD_MS (1000U / FIRMWARE_MOTOR_COMMAND_HZ)

#endif
