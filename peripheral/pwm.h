#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_

#include <cstdint>

class PWMBase {
 public:
   virtual void set_voltage(float v_abc[3]) {}
   virtual void set_vbus(float vbus) {}
   virtual void open_mode() {}
   virtual void brake_mode() {}
   virtual void voltage_mode() {}
   virtual void set_frequency_hz(uint32_t frequency_hz) {}
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_PWM_H_
