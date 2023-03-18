#ifndef UNHUMAN_MOTORLIB_SYSTEM_H_
#define UNHUMAN_MOTORLIB_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void system_init();
void system_run();
void main_loop_interrupt();
void fast_loop_interrupt();
void usb_interrupt();

#ifdef __cplusplus
}

#include <memory>

#include "actuator.h"
#include "communication.h"
#include "logger.h"
#include "parameter_api.h"
#include "round_robin_logger.h"

extern uint32_t t_exec_fastloop;
extern uint32_t t_exec_mainloop;
extern uint32_t t_period_fastloop;
extern uint32_t t_period_mainloop;

void system_maintenance();

class System {
 public:
  // Remove ability to make copies of the object.
  System(System const&) = delete;
  void operator=(System const&) = delete;
  // Create and initialize the first system object.
  static System& init(std::unique_ptr<Actuator>& actuator,
                       std::unique_ptr<CommunicationBase>& communication,
                       std::unique_ptr<Logger>& logger,
                       std::unique_ptr<RoundRobinLogger>& round_robin_logger);
  // Find the singleton instance if needed.
  static System& get_instance();
  // Methods to work with the system.
  void run();
  void main_loop_interrupt();
  void fast_loop_interrupt();
  void log(std::string str);
  std::string get_log();
  char* get_string();
  

  std::unique_ptr<Actuator> actuator_;
  std::unique_ptr<CommunicationBase> communication_;
  std::unique_ptr<Logger> logger_;
  std::unique_ptr<RoundRobinLogger> round_robin_logger_;
  ParameterAPI api_;
  uint32_t count_;

 private:
  // Hide the constructor.
  System();
  System(std::unique_ptr<Actuator> actuator,
          std::unique_ptr<CommunicationBase> communication,
          std::unique_ptr<Logger> logger,
          std::unique_ptr<RoundRobinLogger> round_robin_logger);
  // The singleton instance.
  static System* singleton_;
};

#endif  // __cplusplus
#endif  // UNHUMAN_MOTORLIB_SYSTEM_H_
