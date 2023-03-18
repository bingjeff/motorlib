#ifndef UNHUMAN_MOTORLIB_SENSOR_H_
#define UNHUMAN_MOTORLIB_SENSOR_H_

#include <cstdint>

class SensorBase {
 public:
  virtual void reinit() {}
  virtual void trigger() {}

  // TODO: Consider moving these into the EncoderBase.
  virtual int32_t get_index_pos() const { return 0; }
  virtual bool index_received() const { return false; }

  virtual bool error() { return false; }
};

#endif  // UNHUMAN_MOTORLIB_SENSOR_H_
