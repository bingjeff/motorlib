#pragma once
#include "messages.h"

class TorqueSensor {
 public:
    virtual void init() {}
    virtual void trigger() {}
    virtual float read() = 0;
    virtual void set_param(TorqueSensorParam &param) {
        gain_ = param.gain;
        bias_ = param.bias;
        k_temp_ = param.k_temp;
    }
 protected:
    float gain_ = 0, bias_ = 0, k_temp_ = 0;
};
