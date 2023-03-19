#ifndef UNHUMAN_MOTORLIB_DRIVER_H_
#define UNHUMAN_MOTORLIB_DRIVER_H_

class DriverBase {
 public:
    virtual void enable() { enabled_ = true; }
    virtual void disable() { enabled_ = false; }
    virtual bool is_enabled() const { return enabled_; }
    virtual bool is_faulted() const { return false; }
 private:
    bool enabled_ = false;
};

#endif  // UNHUMAN_MOTORLIB_DRIVER_H_
