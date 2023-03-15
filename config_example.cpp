#include "../param/param_obot_g474.h"
#include "../st_device.h"
#include "torque_sensor.h"
#include "gpio.h"
#include "hall.h"
#include "peripheral/stm32g4/pin_config.h"

using TorqueSensor = TorqueSensorBase;
using MotorEncoder = HallEncoder;
using OutputEncoder = EncoderBase;

extern "C" void SystemClock_Config();
void pin_config_obot_g474_motor_r0();

struct InitCode {
    InitCode() {
      SystemClock_Config();
      pin_config_obot_g474_motor_r0();

      // Setup registers in a unique way, separate from the defaults.
      GPIO_SETL(A, 0, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
      GPIO_SETL(A, 1, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
      GPIO_SETL(A, 2, GPIO::INPUT, GPIO_SPEED::VERY_HIGH, 0);
    }
};

namespace config {
    const uint32_t main_loop_frequency = 10000;    
    const uint32_t pwm_frequency = 50000;
    InitCode init_code;

    GPIO hall_a(*GPIOA, 0);
    GPIO hall_b(*GPIOA, 1);
    GPIO hall_c(*GPIOA, 2);
    HallEncoder motor_encoder(hall_a, hall_b, hall_c);
    TorqueSensor torque_sensor;
    EncoderBase output_encoder;
};

#include "boards/config_obot_g474_motor.cpp"

void config_init() {

}

void config_maintenance() {}
