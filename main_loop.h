#ifndef UNHUMAN_MOTORLIB_MAIN_LOOP_H_
#define UNHUMAN_MOTORLIB_MAIN_LOOP_H_

#include "communication.h"
#include "control_fun.h"
#include "controller/impedance_controller.h"
#include "controller/joint_position_controller.h"
#include "controller/position_controller.h"
#include "controller/state_controller.h"
#include "controller/torque_controller.h"
#include "controller/velocity_controller.h"
#include "cstack.h"
#include "driver.h"
#include "encoder.h"
#include "fast_loop.h"
#include "hardware_brake.h"
#include "led.h"
#include "logger.h"
#include "messages.h"
#include "round_robin_logger.h"
#include "table_interp.h"
#include "torque_sensor.h"

extern "C" {
void system_init();
}
void setup_sleep();
void finish_sleep();


class MainLoop {
 public:
  MainLoop(FastLoop &fast_loop, PositionController &position_controller,
           TorqueController &torque_controller,
           ImpedanceController &impedance_controller,
           VelocityController &velocity_controller,
           StateController &state_controller,
           JointPositionController &joint_position_controller,
           CommunicationBase &communication, LED &led,
           EncoderBase &output_encoder, TorqueSensorBase &torque, DriverBase &driver,
           Logger &logger, RoundRobinLogger &round_robin_logger,
           const MainLoopParam &param, HardwareBrakeBase &brake = no_brake_);
  void init() {}
  void update();

  void set_param(const MainLoopParam &param);
  void set_rollover(float rollover);
  void adjust_output_encoder(float adjustment);
  void set_motor_encoder_bias(float bias);
  const MainLoopStatus &get_status() const;
  void set_started();
  void set_mode(MainControlMode mode);
  bool driver_enable_triggered();
  bool driver_disable_triggered();
  // Set the command from a low priority source other than communication,
  // such as from the System or Actuator classes.
  void set_command(const MotorCommand &command);
  bool is_started() const;
  bool first_command_received() const;

 private:
  LED *led() { return &led_; }
  MainLoopParam param_;
  FastLoop &fast_loop_;
  PositionController &position_controller_;
  TorqueController &torque_controller_;
  ImpedanceController &impedance_controller_;
  VelocityController &velocity_controller_;
  StateController &state_controller_;
  JointPositionController &joint_position_controller_;
  CommunicationBase &communication_;
  LED &led_;
  ReceiveData receive_data_ = {};
  mcu_time host_timestamp_ = {};
  ReceiveData last_receive_data_ = {};
  MotorCommand internal_command_;
  bool internal_command_received_ = false;
  uint64_t count_ = 0;
  uint16_t no_command_ = 0;
  bool safe_mode_ = false;
  bool last_safe_mode_ = false;
  bool started_ = false;
  MainLoopStatus status_ = {};
  MainControlMode mode_ = NO_MODE;
  EncoderBase &output_encoder_;
  float motor_encoder_bias_ = 0;
  TorqueSensorBase &torque_sensor_;
  float dt_ = 0;
  TrajectoryGenerator position_trajectory_generator_;
  uint32_t timestamp_ = 0;
  uint32_t last_timestamp_ = 0;
  uint32_t *reserved0_ = reinterpret_cast<uint32_t *>(&status_.fast_loop.vbus);
  uint32_t *reserved1_ = &timestamp_;
  uint32_t *reserved2_ = &last_timestamp_;
  PChipTable<OUTPUT_ENCODER_TABLE_LENGTH> output_encoder_correction_table_;
  CStack<MainLoopStatus, 2> status_stack_;
  bool first_command_received_ = false;
  DriverBase &driver_;
  Logger &logger_;
  RoundRobinLogger &round_robin_logger_;
  HardwareBrakeBase brake_;
  static HardwareBrakeBase no_brake_;
  volatile bool driver_enable_triggered_ = false;
  volatile bool driver_disable_triggered_ = false;
  uint32_t last_energy_uJ_ = 0;

  friend class System;
  friend class Actuator;
  friend void system_init();
  friend void system_maintenance();
  friend void config_init();
  friend void config_maintenance();
  friend void load_send_data(const MainLoop &main_loop, SendData *const data);
};

void load_send_data(const MainLoop &main_loop, SendData *const data);

#endif  // UNHUMAN_MOTORLIB_MAIN_LOOP_H_
