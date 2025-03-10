#ifndef UNHUMAN_MOTORLIB_ACTUATOR_H_
#define UNHUMAN_MOTORLIB_ACTUATOR_H_

#include "messages.h"

#include "util.h"

extern "C" {
void system_init();
}

class Actuator {
 public:
    Actuator(FastLoop &fast_loop, MainLoop &main_loop, const volatile StartupParam &startup_param) : fast_loop_(fast_loop), main_loop_(main_loop), startup_param_(startup_param) {
      startup_motor_bias_ = startup_param_.motor_encoder_bias;
    }
    void start() {
      if (!startup_param_.no_driver_enable) {
         main_loop_.driver_.enable();
         main_loop_.set_mode(CLEAR_FAULTS); 
      }

      main_loop_.set_rollover(fast_loop_.get_rollover());
      if (!startup_param_.no_zero_current_sensors) {
         // zero current sensors in voltage mode to try to eliminate bias from pwm noise, could also do open mode
         fast_loop_.voltage_mode();
         fast_loop_.zero_current_sensors_on(2);
         ms_delay(2001);
      } else {
         // needs some time to measure bus voltage
         ms_delay(10);
      }
      set_bias();

      if (startup_param_.do_phase_lock) {
         fast_loop_.maintenance();
         fast_loop_.phase_lock_mode(startup_param_.phase_lock_current);
         ms_delay(1000*startup_param_.phase_lock_duration);
      }
      fast_loop_.maintenance();  // TODO better way than calling this to update zero pos
      fast_loop_.voltage_mode();
      main_loop_.set_mode(startup_param_.startup_mode);
      fast_loop_.set_iq_des(0);
      main_loop_.set_started();
    }
    void enable_driver() {
         if (main_loop_.status_.fast_loop.vbus > main_loop_.param_.vbus_min && 
            main_loop_.status_.fast_loop.vbus < main_loop_.param_.vbus_max) {
            main_loop_.driver_.enable();
         }
         main_loop_.set_mode(CLEAR_FAULTS);
    }
    void maintenance() {
      fast_loop_.maintenance();
      if (main_loop_.driver_enable_triggered()) {
         enable_driver();
      }
      if (main_loop_.driver_disable_triggered()) {
         ms_delay(10);
         main_loop_.driver_.disable();
      }
    }
    void set_bias() {
      MainLoopStatus status = main_loop_.get_status();
      if (status.output_position > startup_param_.output_encoder_rollover) {
         main_loop_.adjust_output_encoder(-2*M_PI);
         status.output_position -= 2*M_PI;
      }
      switch(startup_param_.motor_encoder_startup) {
         default:
         case StartupParam::ENCODER_ZERO:
            break;
         case StartupParam::ENCODER_BIAS: {
            main_loop_.set_motor_encoder_bias(-status.motor_position + startup_motor_bias_);
            break;
         }
         case StartupParam::ENCODER_BIAS_FROM_OUTPUT: {
            main_loop_.set_motor_encoder_bias(status.output_position * startup_param_.gear_ratio 
              - status.fast_loop.motor_position.position + startup_motor_bias_);
            break;
         }
         case StartupParam::ENCODER_BIAS_FROM_OUTPUT_WITH_MOTOR_CORRECTION: {
            float round_by = 2*M_PI*(startup_param_.num_encoder_poles == 0 ? 1 : startup_param_.num_encoder_poles);
            float motor_bias_from_output = status.output_position * startup_param_.gear_ratio 
              - (status.fast_loop.motor_position.position);
            float motor_bias_rounded = roundf(motor_bias_from_output/round_by)*(round_by) + startup_motor_bias_;
            logger.log("Encoder bias from output with correction");
            logger.log_printf("Output position: %f, motor_position: %f, motor mechanical position: %f", 
               status.output_position, status.fast_loop.motor_position.position, status.fast_loop.motor_mechanical_position);
            logger.log_printf("Encoder bias, round_by: %f, motor_bias_from_output: %f, motor_bias_rounded: %f",
               round_by, motor_bias_from_output, motor_bias_rounded);
            main_loop_.set_motor_encoder_bias(motor_bias_rounded);
            break;
         }
         case StartupParam::ENCODER_BIAS_FROM_OUTPUT_WITH_TORQUE_AND_MOTOR_CORRECTION: {
            // todo fix as above
            float round_by = 2*M_PI*(startup_param_.num_encoder_poles == 0 ? 1 : startup_param_.num_encoder_poles);
            float motor_bias_from_output = (status.output_position - status.torque*startup_param_.transmission_stiffness) * startup_param_.gear_ratio 
              - (status.fast_loop.motor_position.position + startup_motor_bias_);
            float motor_bias_rounded = roundf(motor_bias_from_output*round_by)/(round_by);
            main_loop_.set_motor_encoder_bias(motor_bias_rounded);
            break;
         }
      }
   }
private:
    FastLoop &fast_loop_;
    MainLoop &main_loop_;
    const volatile StartupParam &startup_param_;
    float startup_motor_bias_;

    friend class System;
    friend void system_init();
    friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_ACTUATOR_H_
