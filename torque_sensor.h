#ifndef UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_
#define UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_

#include "messages.h"
#include "sensor.h"

class TorqueSensorBase : public SensorBase {
 public:
  // TODO: Make one return type for init, make it pure virtual, and put in sensorbase.
  bool init() { return true; }
  // Read data directly from the sensor.
  virtual float read() { return 0; }
  // Returns the last read result without triggering an update of the sensor.
  virtual float get_value() const { return 0; }
  virtual void set_param(const TorqueSensorParam &param) {
    gain_ = param.gain;
    bias_ = param.bias;
    k_temp_ = param.k_temp;
  }

  float gain_ = 0;
  float bias_ = 0;
  float k_temp_ = 0;
  float torque_ = 0;
};

#endif  // UNHUMAN_MOTORLIB_TORQUE_SENSOR_H_
