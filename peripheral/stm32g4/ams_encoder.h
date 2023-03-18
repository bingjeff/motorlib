#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_AMS_ENCODER_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_AMS_ENCODER_H_

#include "../../encoder.h"
#include "../st_device.h"


class GPIO;

class AMSEncoder : public EncoderBase {
 public:
    AMSEncoder(SPI_TypeDef &regs, GPIO &gpio_cs) : EncoderBase(), regs_(regs), gpio_cs_(gpio_cs) {} 
    bool init() override { return true; }
    int32_t read() override __attribute__((section (".ccmram")));
    int32_t get_value() const override __attribute__((section (".ccmram")));
    void trigger() override __attribute__((section (".ccmram")));
    bool index_received() const override { return true; }
 protected:
    SPI_TypeDef &regs_;
    GPIO &gpio_cs_;
    uint16_t last_data_ = 0;
    int32_t count_ = 0;
    uint8_t error_count_ = 255;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32G4_AMS_ENCODER_H_
