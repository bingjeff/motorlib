#ifndef UNHUMAN_MOTORLIB_ACTUATOR_H_
#define UNHUMAN_MOTORLIB_ACTUATOR_H_

#include "fast_loop.h"
#include "main_loop.h"
#include "messages.h"

extern "C" {
void system_init();
}

class Actuator {
 public:
  Actuator(FastLoop &fast_loop, MainLoop &main_loop,
           const volatile StartupParam &startup_param)
      : fast_loop_(fast_loop),
        main_loop_(main_loop),
        startup_param_(startup_param) {}
  void start();
  void enable_driver();
  void maintenance();
  void set_bias();

 private:
  FastLoop &fast_loop_;
  MainLoop &main_loop_;
  const volatile StartupParam &startup_param_;

  friend class System;
  friend void system_init();
  friend void config_init();
};

#endif  // UNHUMAN_MOTORLIB_ACTUATOR_H_
