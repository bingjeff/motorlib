#ifndef UNHUMAN_MOTORLIB_BUCK_MESSAGES_BUCK_H_
#define UNHUMAN_MOTORLIB_BUCK_MESSAGES_BUCK_H_

#include "../messages.h"

struct BuckStatus {
    float v_bus, v_out;
    float i_bus, i_out;
};

struct FastLoopBuckParam {
    float i_bus_a_per_count;
    float i_bus_bias_a;
    float v_bus_v_per_count;
    float i_out_a_per_count;
    float v_out_v_per_count;
    PIParam pi_param;
};

#endif  // UNHUMAN_MOTORLIB_BUCK_MESSAGES_BUCK_H_
