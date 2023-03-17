#ifndef UNHUMAN_MOTORLIB_FAST_LOOP_H_
#define UNHUMAN_MOTORLIB_FAST_LOOP_H_

#include <cmath>
#include <cstdint>

#include "control_fun.h"
#include "cstack.h"
#include "encoder.h"
#include "foc.h"
#include "messages.h"
#include "peripheral/pwm.h"
#include "table_interp.h"
#include "util.h"


class FastLoop {
 public:
  FastLoop(int32_t frequency_hz, PWMBase &pwm, EncoderBase &motor_encoder,
           const FastLoopParam &param, volatile uint32_t *const i_a_dr,
           volatile uint32_t *const i_b_dr, volatile uint32_t *const i_c_dr,
           volatile uint32_t *const v_bus_dr);
  ~FastLoop();

  void update() __attribute__((section(".ccmram")));
  void maintenance();
  void set_id_des(float id);
  void set_iq_des(float iq);
  void set_vq_des(float vq);
  void set_tuning_amplitude(float amplitude);
  void set_tuning_frequency(float frequency);
  void set_tuning_chirp(bool on, float chirp_rate);
  void set_tuning_bias(float bias);
  void set_tuning_square(bool square = true);
  void set_stepper_position(float position);
  void set_stepper_velocity(float velocity);
  void set_reserved(float reserved);
  void phase_lock_mode(float id);
  void current_mode();
  void current_tuning_mode();
  void voltage_mode();
  void stepper_mode();
  void brake_mode();
  void open_mode();
  void set_param(const FastLoopParam &fast_loop_param);
  const FastLoopStatus &get_status() const;
  void store_status();
  void zero_current_sensors();
  void set_phase_mode();
  void set_phase_mode(uint8_t phase_mode);
  uint8_t get_phase_mode();
  float get_rollover() const;
  void beep_on(float t_seconds = 1);
  void beep_off();
  void zero_current_sensors_on(float t_seconds = 1);
  void zero_current_sensors_off();
  bool motor_encoder_error();
  void trigger_status_log();

 private:
  FastLoopParam param_;
  FOC *foc_;
  PWMBase &pwm_;
  enum {
    OPEN_MODE,
    BRAKE_MODE,
    CURRENT_MODE,
    PHASE_LOCK_MODE,
    VOLTAGE_MODE,
    CURRENT_TUNING_MODE,
    STEPPER_TUNING_MODE
  } mode_ = CURRENT_MODE;

  int32_t motor_enc;
  int32_t last_motor_enc = 0;
  float motor_position_ = 0;
  float motor_position_filtered_ = 0;
  float motor_velocity = 0;
  float motor_velocity_filtered = 0;
  float alpha = 0.001;
  float alpha10 = 1;      // 0.3859;   // 1/10 cutoff frequency
  float phase_mode_ = 1;  // 1: standard or -1: two wires switched
  float phase_mode_desired_ = 1;
  int32_t motor_mechanical_position_ = 0;

  float iq_des = 0;
  float id_des = 0;
  float iq_des_gain_ = 1;
  volatile uint16_t adc1, adc2, adc3;
  FOCCommand foc_command_ = {};

  int32_t motor_index_pos_ = 0;
  bool motor_index_pos_set_ = false;
  int32_t motor_electrical_zero_pos_;
  float motor_index_electrical_offset_measured_ = NAN;
  float inv_motor_encoder_cpr_;
  int32_t frequency_hz_ = 100000;
  float alpha_zero_ = 0.0002;
  float v_bus_ = 12;
  mcu_time timestamp_;
  EncoderBase &motor_encoder_;
  float reserved_ = 0;
  mcu_time last_timestamp_ = 0;
  float dt_ = 0;
  float phi_ = 0;
  float tuning_amplitude_ = 0;
  float tuning_frequency_ = 0;
  float tuning_bias_ = 0;
  bool tuning_square_ = false;
  float chirp_rate_ = 0;
  bool current_tuning_chirp_ = false;
  KahanSum chirp_frequency_;
  float stepper_position_ = 0;
  float stepper_velocity_ = 0;
  int32_t motor_enc_wrap_ = 0;
  volatile uint32_t *const i_a_dr_;
  volatile uint32_t *const i_b_dr_;
  volatile uint32_t *const i_c_dr_;
  volatile uint32_t *const v_bus_dr_;
  PChipTable<MOTOR_ENCODER_TABLE_LENGTH> motor_correction_table_;
  PChipTable<COGGING_TABLE_SIZE> cogging_correction_table_;
  CStack<FastLoopStatus, 100> status_;
  CStack<FastLoopStatus, 100> status_log_;
  bool beep_ = false;
  uint32_t beep_end_ = 0;
  bool zero_current_sensors_ = false;
  uint32_t zero_current_sensors_end_ = 0;
  float phi_beep_ = 0;
  uint32_t energy_uJ_ = 0;

  friend class System;
};

#endif  // UNHUMAN_MOTORLIB_FAST_LOOP_H_
