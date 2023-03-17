#include "fast_loop.h"

#include "sincos.h"

FastLoop::FastLoop(int32_t frequency_hz, PWMBase &pwm, EncoderBase &motor_encoder,
                   const FastLoopParam &param, volatile uint32_t *const i_a_dr,
                   volatile uint32_t *const i_b_dr,
                   volatile uint32_t *const i_c_dr,
                   volatile uint32_t *const v_bus_dr)
    : param_(param),
      pwm_(pwm),
      motor_encoder_(motor_encoder),
      i_a_dr_(i_a_dr),
      i_b_dr_(i_b_dr),
      i_c_dr_(i_c_dr),
      v_bus_dr_(v_bus_dr),
      motor_correction_table_(param_.motor_encoder.table),
      cogging_correction_table_(param_.cogging.table) {
  frequency_hz_ = frequency_hz;
  float dt = 1.0f / frequency_hz;
  foc_ = new FOC(dt);
  set_param(param);
#ifdef END_TRIGGER_MOTOR_ENCODER
  encoder_.trigger();
#endif
}
FastLoop::~FastLoop() { delete foc_; }
void FastLoop::update() {
  // trigger encoder read
#ifndef END_TRIGGER_MOTOR_ENCODER
  // probably don't use end trigger on a shared spi bus
  motor_encoder_.trigger();
#endif

  timestamp_ = get_clock();

  // get ADC
  adc1 = *i_a_dr_;
  adc2 = *i_b_dr_;
  adc3 = *i_c_dr_;
  foc_command_.measured.i_a = param_.adc1_gain * (adc1 - 2048) - param_.ia_bias;
  foc_command_.measured.i_b = param_.adc2_gain * (adc2 - 2048) - param_.ib_bias;
  foc_command_.measured.i_c = param_.adc3_gain * (adc3 - 2048) - param_.ic_bias;

  // get encoder value, may wait a little
  motor_enc = motor_encoder_.read();
  int32_t motor_enc_diff = motor_enc - last_motor_enc;
  motor_enc_wrap_ =
      wrap1(motor_enc_wrap_ + motor_enc_diff, param_.motor_encoder.rollover);
  motor_mechanical_position_ = (motor_enc_wrap_ - motor_index_pos_);
  float motor_x = motor_mechanical_position_ * inv_motor_encoder_cpr_;

  motor_position_ =
      param_.motor_encoder.dir *
      (2 * (float)M_PI * inv_motor_encoder_cpr_ * motor_enc_wrap_ +
       motor_index_pos_set_ * motor_correction_table_.table_interp(motor_x));
  // TODO: Re-enable filtering:
  // (1-alpha10)*motor_position_filtered_ + alpha10*motor_position_
  motor_position_filtered_ = motor_position_;
  motor_velocity = param_.motor_encoder.dir * (motor_enc_diff) *
                   (2 * (float)M_PI * inv_motor_encoder_cpr_ * frequency_hz_);
  motor_velocity_filtered =
      (1 - alpha) * motor_velocity_filtered + alpha * motor_velocity;
  last_motor_enc = motor_enc;

  // cogging compensation, interpolate in the table
  float iq_ff =
      param_.cogging.gain * cogging_correction_table_.table_interp(motor_x);

  if (mode_ == CURRENT_TUNING_MODE) {
    // only works down to frequencies of .047 Hz, could use kahansum to go
    // slower
    if (current_tuning_chirp_) {
      tuning_frequency_ = chirp_frequency_.add(chirp_rate_ * dt_);
    }
    phi_ += 2 * (float)M_PI * fabsf(tuning_frequency_) *
            dt_;  // use id des to set frequency
    if (phi_ > 2 * (float)M_PI) {
      phi_ -= 2 * (float)M_PI;
    }
    Sincos sincos;
    sincos = sincos1(phi_);
    iq_des =
        tuning_bias_ +
        tuning_amplitude_ * (tuning_square_ ? fsignf(sincos.sin) : sincos.sin);
  }

  if (beep_) {
    if ((int32_t)(get_clock() - beep_end_) > 0) {
      beep_ = false;
    } else {
      phi_beep_ += 2 * (float)M_PI * fabsf(param_.beep_frequency) * dt_;
      if (phi_beep_ > 2 * (float)M_PI) {
        phi_beep_ -= 2 * (float)M_PI;
      }
      Sincos sincos = sincos1(phi_beep_);
      iq_ff += param_.beep_amplitude * fsignf(sincos.sin);
    }
  }

  // update FOC
  foc_command_.measured.motor_encoder =
      phase_mode_ * (motor_enc_wrap_ - motor_electrical_zero_pos_) *
      (2 * (float)M_PI * inv_motor_encoder_cpr_);
  foc_command_.desired.i_q = iq_des_gain_ * (iq_des + iq_ff);

  if (mode_ == STEPPER_TUNING_MODE) {
    foc_command_.measured.motor_encoder = stepper_position_;
    motor_position_filtered_ = stepper_position_;
    stepper_position_ += stepper_velocity_ * dt_;
    stepper_position_ = wrap1(stepper_position_, static_cast<float>(2 * M_PI));
  }

  FOCStatus *foc_status = foc_->step(foc_command_);

  // output pwm
  pwm_.set_voltage(&foc_status->command.v_a);

  dt_ = (timestamp_ - last_timestamp_) * (float)(1.0f / CPU_FREQUENCY_HZ);
  last_timestamp_ = timestamp_;

  if (zero_current_sensors_) {
    if ((int32_t)(get_clock() - zero_current_sensors_end_) > 0) {
      zero_current_sensors_ = false;
    } else {
      zero_current_sensors();
    }
  }
  store_status();
#ifdef END_TRIGGER_MOTOR_ENCODER
  encoder_.trigger();
#endif
}
void FastLoop::maintenance() {
  if (motor_encoder_.index_received() && !motor_index_pos_set_) {
    motor_index_pos_ = motor_encoder_.get_index_pos();
    if (param_.motor_encoder.use_index_electrical_offset_pos) {
      // motor_index_electrical_offset_pos is the value of an electrical zero
      // minus the index position motor_electrical_zero_pos is the offset to the
      // initial encoder value
      motor_electrical_zero_pos_ =
          param_.motor_encoder.index_electrical_offset_pos + motor_index_pos_;
    }
    motor_index_pos_set_ = true;
  }

  if (mode_ == PHASE_LOCK_MODE) {
    motor_electrical_zero_pos_ = motor_encoder_.get_value();
    if (motor_encoder_.index_received()) {
      motor_index_pos_ = motor_encoder_.get_index_pos();
      int32_t index_offset = motor_electrical_zero_pos_ - motor_index_pos_;
      if (index_offset >= 0) {
        motor_index_electrical_offset_measured_ =
            index_offset %
            (param_.motor_encoder.cpr / (uint8_t)param_.foc_param.num_poles);
      } else {
        int32_t m =
            (param_.motor_encoder.cpr / (uint8_t)param_.foc_param.num_poles);
        motor_index_electrical_offset_measured_ =
            index_offset - m * ((index_offset / m) - 1);
      }
    }
  }

  v_bus_ = *v_bus_dr_ * param_.vbus_gain;
  pwm_.set_vbus(fmaxf(7, v_bus_));
}
void FastLoop::set_id_des(float id) { foc_command_.desired.i_d = id; }
void FastLoop::set_iq_des(float iq) {
  if (mode_ == CURRENT_MODE) iq_des = iq;
}
void FastLoop::set_vq_des(float vq) { foc_command_.desired.v_q = vq; }
void FastLoop::set_tuning_amplitude(float amplitude) {
  tuning_amplitude_ = amplitude;
}
void FastLoop::set_tuning_frequency(float frequency) {
  tuning_frequency_ = frequency;
}
void FastLoop::set_tuning_chirp(bool on, float chirp_rate) {
  current_tuning_chirp_ = on;
  chirp_rate_ = chirp_rate;
  chirp_frequency_.init(0);
}
void FastLoop::set_tuning_bias(float bias) { tuning_bias_ = bias; }
void FastLoop::set_tuning_square(bool square) {
  tuning_square_ = square;
}
void FastLoop::set_stepper_position(float position) {
  stepper_position_ = position;
}
void FastLoop::set_stepper_velocity(float velocity) {
  stepper_velocity_ = velocity;
}
void FastLoop::set_reserved(float reserved) { reserved_ = reserved; }
void FastLoop::phase_lock_mode(float id) {
  phase_mode_ = 0;
  set_id_des(id);
  iq_des_gain_ = 0;
  pwm_.voltage_mode();
  foc_->current_mode();
  mode_ = PHASE_LOCK_MODE;
}
void FastLoop::current_mode() {
  phase_mode_ = phase_mode_desired_;
  set_id_des(0);
  iq_des_gain_ = 1;
  pwm_.voltage_mode();
  foc_->current_mode();
  mode_ = CURRENT_MODE;
}
void FastLoop::current_tuning_mode() {
  current_mode();
  phi_ = 0;
  mode_ = CURRENT_TUNING_MODE;
}
void FastLoop::voltage_mode() {
  phase_mode_ = phase_mode_desired_;
  pwm_.voltage_mode();
  foc_->voltage_mode();
  mode_ = VOLTAGE_MODE;
}
void FastLoop::stepper_mode() {
  phase_mode_ = phase_mode_desired_;
  pwm_.voltage_mode();
  foc_->voltage_mode();
  mode_ = STEPPER_TUNING_MODE;
}
void FastLoop::brake_mode() {
  pwm_.brake_mode();
  foc_->voltage_mode();
  mode_ = BRAKE_MODE;
}
void FastLoop::open_mode() {
  pwm_.open_mode();
  foc_->voltage_mode();
  mode_ = OPEN_MODE;
}
void FastLoop::set_param(const FastLoopParam &fast_loop_param) {
  param_ = fast_loop_param;
  foc_->set_param(param_.foc_param);
  set_phase_mode();
  inv_motor_encoder_cpr_ =
      param_.motor_encoder.cpr != 0 ? 1.f / param_.motor_encoder.cpr : 0;
}
const FastLoopStatus &FastLoop::get_status() const { return status_.top(); }
void FastLoop::store_status() {
  FastLoopStatus &s = status_.next();
  s.motor_mechanical_position = motor_mechanical_position_;
  s.motor_position.position = motor_position_filtered_;
  s.motor_position.raw = motor_enc;
  s.timestamp = timestamp_;
  s.vbus = v_bus_;
  s.foc_command = foc_command_;
  s.power = s.foc_status.command.v_d * s.foc_status.measured.i_d +
            s.foc_status.command.v_q * s.foc_status.measured.i_q;
  int32_t energy = s.power * dt_ * 1e6;
  energy_uJ_ += (uint32_t)energy;
  s.energy_uJ = energy_uJ_;
  foc_->get_status(&s.foc_status);
  status_.finish();
}

void FastLoop::zero_current_sensors() {
  param_.ia_bias = (1 - alpha_zero_) * param_.ia_bias +
                   alpha_zero_ * param_.adc1_gain * (adc1 - 2048);
  param_.ib_bias = (1 - alpha_zero_) * param_.ib_bias +
                   alpha_zero_ * param_.adc2_gain * (adc2 - 2048);
  param_.ic_bias = (1 - alpha_zero_) * param_.ic_bias +
                   alpha_zero_ * param_.adc3_gain * (adc3 - 2048);
}
void FastLoop::set_phase_mode() {
  phase_mode_desired_ = param_.phase_mode == 0 ? 1 : -1;
}
void FastLoop::set_phase_mode(uint8_t phase_mode) {
  param_.phase_mode = phase_mode;
  set_phase_mode();
}
uint8_t FastLoop::get_phase_mode() { return param_.phase_mode; }
float FastLoop::get_rollover() const {
  return 2 * M_PI * inv_motor_encoder_cpr_ * param_.motor_encoder.rollover;
}
void FastLoop::beep_on(float t_seconds) {
  beep_ = true;
  beep_end_ = get_clock() + t_seconds * CPU_FREQUENCY_HZ;
}
void FastLoop::beep_off() { beep_ = false; }

void FastLoop::zero_current_sensors_on(float t_seconds) {
  zero_current_sensors_ = true;
  zero_current_sensors_end_ = get_clock() + t_seconds * CPU_FREQUENCY_HZ;
}
void FastLoop::zero_current_sensors_off() { zero_current_sensors_ = false; }
bool FastLoop::motor_encoder_error() { return motor_encoder_.error(); }
void FastLoop::trigger_status_log() { status_log_.copy(status_); }