#ifndef UNHUMAN_MOTORLIB_AKSIM2_ENCODER_H_
#define UNHUMAN_MOTORLIB_AKSIM2_ENCODER_H_

#include "encoder.h"
#include <cmath>

uint8_t CRC_BiSS_43_24bit(uint32_t w_InputData);

template<uint8_t nbits_>
class Aksim2Encoder : public EncoderBase {
 public:
    union Diag {
        struct {
            uint8_t crc6:6;
            uint8_t warn:1;
            uint8_t err:1;
        };
        uint8_t word;
    };
    union DiagProcessed {   // 0b111 is good
        struct {
            uint8_t crc6:1;
            uint8_t warn:1;
            uint8_t err:1;
            uint8_t resv:5;
        };
        uint8_t word;
    };
    // BISS, 5 MHz max
    Aksim2Encoder(SPIDMA &spi_dma) : EncoderBase(), spi_dma_(spi_dma) {}
    void trigger() {
        spi_dma_.start_readwrite(data_out_, data_in_, length_);
    }
    int32_t read() {
        spi_dma_.finish_readwrite();
        // 17 bits of nothing, then number of data bits 18-20, then error bit, warn bit, and crc6
        // 17+20+2+6 = 45 -> 6 bytes
        // raw value will be 31 bits with a leading zero
        raw_value_ = data_in_[2] << 24 | data_in_[3] << 16 | data_in_[4] << 8 | data_in_[5];
        Diag diag;
        diag.word = (raw_value_ >> (31 - 8 - nbits_)) & 0xFF;
        diag_.err = diag.err;
        diag_.warn = diag.warn;
        crc_calc_ = ~CRC_BiSS_43_24bit(raw_value_ >> (31 - 2 - nbits_)) & 0x3F; // crc of data plus 2 status bits
        diag_.crc6 = crc_calc_ == diag.crc6;
        if (!diag_.err) {
            diag_err_count_++;
        }
        if (!diag_.warn) {
            diag_warn_count_++;
        }
        if (!diag_.crc6) {
            crc_err_count_++;
        }

        if (diag_.crc6 && diag_.err) {
            uint32_t shift_value = (raw_value_ >> (31 - nbits_)) << (32 - nbits_);
            int32_t diff = (int32_t) (shift_value - last_shift_value_) >> (32 - nbits_);
            last_shift_value_ = shift_value;
            value_ += diff;
        }

        diag_raw_ = diag;
        return get_value();
    }
    int32_t get_value() const { return value_; }
    bool index_received() { return true; }
    DiagProcessed diag_ = {};
    uint8_t crc_calc_ = {};
    Diag diag_raw_ = {};
    uint32_t crc_err_count_ = 0;
    uint32_t diag_err_count_ = 0;
    uint32_t diag_warn_count_ = 0;
    uint32_t raw_value_ = 0;


 private:
    SPIDMA &spi_dma_;
    static const uint8_t length_ = 6;
    uint8_t data_out_[length_] = {};
    uint8_t data_in_[length_] = {};
    int32_t value_ = 0;
    uint32_t last_shift_value_ = 0;

    friend void config_init();
};


// CRC6 calc from RLS CRCD01
uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};
/*32-bit input data, right alignment, Calculation over 24 bits (mult. of 6) */
uint8_t CRC_BiSS_43_24bit (uint32_t w_InputData)
{
 uint8_t b_Index = 0;
 uint8_t b_CRC = 0;

 b_Index = (uint8_t )(((uint32_t)w_InputData >> 18u) & 0x0000003Fu);

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )((uint32_t)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 

#endif  // UNHUMAN_MOTORLIB_AKSIM2_ENCODER_H_
