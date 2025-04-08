#ifndef LIMIT_SWITCH_H_
#define LIMIT_SWITCH_H_

#include <chrono>
#include <functional>
#include <iostream>

class LimitSwitch {
 public:
  // A callback type that is called when the switch state changes.
  // The boolean parameter is true if the switch is pressed.
  using StateChangeCallback = std::function<void(bool)>;

  // Constructs a LimitSwitch for a given GPIO pin.
  // debounce_threshold_ms specifies the minimum time between state changes.
  LimitSwitch(int gpio_pin, int debounce_threshold_ms = 30);

  // Destructor.
  ~LimitSwitch();

  // Delete copy constructor and assignment operator.
  LimitSwitch(const LimitSwitch&) = delete;
  LimitSwitch& operator=(const LimitSwitch&) = delete;

  // Processes a GPIO alert.
  // 'level' is the current reading from the GPIO (LOW means pressed with pull-up).
  // 'tick' is the tick count from the alert.
  void HandleAlert(int level, uint32_t tick);

  // Returns true if the switch is currently pressed.
  bool IsPressed() const;

  // Sets a callback function to be called when the switch state changes.
  void SetStateChangeCallback(StateChangeCallback callback);

 private:
  int gpio_pin_;
  int debounce_threshold_ms_;
  bool last_state_;
  std::chrono::steady_clock::time_point last_debounce_time_;
  bool pressed_;
  StateChangeCallback callback_;
};

#endif  // LIMIT_SWITCH_H_
