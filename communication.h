#ifndef UNHUMAN_MOTORLIB_COMMUNICATION_H_
#define UNHUMAN_MOTORLIB_COMMUNICATION_H_

#include "messages.h"

class CommunicationBase {
 public:
  virtual int receive_data(ReceiveData *const data) { return 0; }
  virtual bool receive_ready() { return false; }

  virtual void send_data(const SendData &data) {}
  virtual bool send_acknowledged() { return false; }

  virtual int receive_string(char *const string) { return 0; }
  virtual bool send_string(const char *const string, uint16_t length) { return false; }
  virtual bool send_string_active() const { return false; }
  virtual void cancel_send_string() {}
};

#endif  // UNHUMAN_MOTORLIB_COMMUNICATION_H_
