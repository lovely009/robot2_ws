// BMI088 ACC_CONF 的 100 Hz 编码必须遵循芯片寄存器契约；错误的保留值会让 ACC DRDY 停止。

#include "bmi088_reg.h"

constexpr auto kAccConf100Hz =
    BMI088_ACC_CONF_MUST_Set | BMI088_ACC_NORMAL | BMI088_ACC_100_HZ;
static_assert(kAccConf100Hz == 0xA8U,
              "BMI088 100 Hz ACC_CONF must use the datasheet ODR code");

int main() { return 0; }
