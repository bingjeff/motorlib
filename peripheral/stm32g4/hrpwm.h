#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_

#include <cstdint>
#include <vector>

#include "../../../boost_g474/st_device.h"
#include "../pwm.h"

extern "C" {
void system_init();
}

class HRPWM final : public PWMBase {
 public:
  HRPWM(uint32_t frequency_hz, HRTIM_TypeDef &regs, uint8_t ch_a, uint8_t ch_b,
        uint8_t ch_c, bool pwm3_mode = false, uint16_t deadtime_ns = 50,
        uint16_t min_off_ns = 0, uint16_t min_on_ns = 0);

  void init();
  void set_deadtime(uint16_t deadtime_ns);
  void set_frequency_hz(uint32_t frequency_hz, uint16_t min_off_ns,
                        uint16_t min_on_ns);

  void set_voltage(float v_abc[3]) override __attribute__((section(".ccmram")));
  void set_vbus(float vbus) override;
  void open_mode() override;
  void brake_mode() override;
  void voltage_mode() override;
  void set_frequency_hz(uint32_t frequency_hz) override;

 private:
  uint16_t period_, half_period_;
  HRTIM_TypeDef &regs_;
  volatile uint32_t &pwm_a_, &pwm_b_, &pwm_c_;
  uint8_t ch_a_, ch_b_, ch_c_;
  float v_to_pwm_;
  bool pwm3_mode_;
  uint16_t deadtime_ns_;
  float pwm_min_ = 0;
  float pwm_max_;
  int prescaler_ = 32;
  float count_per_ns_;

  friend void config_init();
  friend void system_init();
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_HRPWM_H_
