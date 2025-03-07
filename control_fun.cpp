#include "control_fun.h"
//#include "hal_fun.h"
#include <cmath>

float fsat(float a, float sat) {
    // Slow version
    // if (a > sat) {
    //     return sat;
    // } else if (a < -sat) {
    //     return -sat;
    // } else {
    //     return a;
    // }

    // Doesn't completely optimize to branchless instructions
    // return a>sat ? sat : (a<-sat ? -sat : a);

    // Optimizes to branchless
    float b = a>sat ? sat : a;
    b = b<-sat ? -sat : b;
    return b;

    // Not so great
    // float s = a>0 ? sat : -sat;
    // return std::abs(a)-sat>0 ? s : a;

    // Assembly
    // asm("vcmpe.f32 %[a], %[sat]\n\t"
    //     "vmrs APSR_nzcv, fpscr\n\t"
    //     "it gt\n\t"
    //     "vmovgt.f32 %[a], %[sat]\n\t"
    //     "vneg.f32 %[sat], %[sat]\n\t"
    //     "vcmpe.f32 %[a], %[sat]\n\t"
    //     "vmrs APSR_nzcv, fpscr\n\t"
    //     "it lt\n\t"
    //     "vmovlt.f32 %[a], %[sat]\n\t"
    //     :
    //     : [a] "t" (a), [sat] "t" (sat));
    // return a;
}

float fsignf(float a) {
    return a>=0 ? 1 : -1;
}

void Hysteresis::set_hysteresis(float value) {
    hysteresis_ = value;
}

float Hysteresis::step(float value) {
    if (value - value_ > hysteresis_) {
        value_ = value - hysteresis_;
    } else if (value - value_ < -hysteresis_) {
        value_ = value + hysteresis_;
    }
    return value_;
}

void PIController::set_param(const PIParam &pi_param) {
    ki_ = pi_param.ki;
    kp_ = pi_param.kp;
    ki_limit_ = pi_param.ki_limit;
    command_max_ = pi_param.command_max;
}

float PIController::step(float desired, float measured) {
    float error = desired - measured;
    ki_sum_ += ki_ * error;
    ki_sum_ = fsat(ki_sum_, ki_limit_);
    return fsat(kp_*error + ki_sum_, command_max_);
}

void PIDController::set_param(const PIDParam &param) {
    ki_ = param.ki;
    kp_ = param.kp;
    ki_limit_ = param.ki_limit;
    kd_ = param.kd;
    command_max_ = param.command_max;
    velocity_filter_.set_frequency(param.velocity_filter_frequency_hz);
    output_filter_.set_frequency(param.output_filter_frequency_hz);
    hysteresis_.set_hysteresis(command_max_/kp_);
}

float PIDController::step(float desired, float velocity_desired, float measured, float velocity_limit) {
    // PID controller with formula
    // out = (ki/s + kp + kd*s) * error
    // with s*error given by velocity_desired - velocity measured from measured with internal 2nd order filter
    // with saturation on ki/s * error: ki_limit
    // saturation on out: command_max
    // output filter is first order filter for lead controller option
    // velocity limit: velocity_limit on command - not if not tracking velocity can exceed this

    rate_limit_.set_limit(fabsf(velocity_limit*dt_));
    
    // proxy is a prefix for a limited desired value
    float proxy_desired = rate_limit_.step(desired);
    float proxy_velocity_desired = fsat(velocity_desired, velocity_limit);
    error_ = proxy_desired - measured;
    velocity_measured_ = velocity_filter_.update((measured - measured_last_)/dt_);
    float error_dot = proxy_velocity_desired - velocity_measured_;
    measured_last_ = measured;
    ki_sum_ += ki_ * dt_ * error_;
    ki_sum_ = fsat(ki_sum_, ki_limit_);
    float filtered_out = output_filter_.update(kp_*error_ + ki_sum_ + kd_*error_dot);
    return fsat(filtered_out, command_max_);
}

float PIDWrapController::step(float desired, float velocity_desired, float measured, float velocity_limit) {
    // PID controller with formula
    // out = (ki/s + kp + kd*s) * error
    // with s*error given by velocity_desired - velocity measured from measured with internal 2nd order filter
    // with saturation on ki/s * error: ki_limit
    // saturation on out: command_max
    // output filter is first order filter for lead controller option
    // velocity limit: velocity_limit on command - not if not tracking velocity can exceed this
    // rollover allows for going to the closest position if through rollover, 
    //      e.g. rollover = 2*pi, measured = 1.5*pi, desired = -1.5*pi, motor will go through rollover
    //      if desired is > 2*pi motor will still calculate error correctly, but will spin forever

    rate_limit_.set_limit(fabsf(velocity_limit*dt_));
    float desired_wrap = wrap1(desired, rollover_);
    // wrap the rate limiter if necessary
    if (fabsf(desired_wrap - rate_limit_.get_value()) > 1.5*rollover_) {
        rate_limit_.init(rate_limit_.get_value() + desired_wrap - desired, rate_limit_.get_velocity());
    }
    
    // proxy is a prefix for a limited desired value
    float proxy_desired = rate_limit_.step(desired_wrap);
    //float proxy_wrap = wrap1(proxy_desired, rollover_);
    //rate_limit_.init(proxy_wrap, rate_limit_.get_velocity());
   // float proxy_dot_desired = fsat(velocity_desired, fabsf(rate_limit_.get_velocity()/dt_));
    error_ = wrap1_diff(proxy_desired, measured, rollover_);
    float error_dot = velocity_filter_.update((error_ - error_last_)/dt_);
    error_last_ = error_;
    ki_sum_ += ki_ * dt_ * error_;
    ki_sum_ = fsat(ki_sum_, ki_limit_);
    float filtered_out = output_filter_.update(kp_*error_ + ki_sum_ + kd_*error_dot);
    return fsat(filtered_out, command_max_);
}

float PIDDeadbandController::step(float desired, float velocity_desired, float deadband, float measured, float velocity_limit) {
    float desired_with_deadband = fsignf(desired-measured)*fmaxf(fabsf(desired-measured) - deadband, 0) + measured;
    return PIDController::step(desired_with_deadband, velocity_desired, measured, velocity_limit);
}