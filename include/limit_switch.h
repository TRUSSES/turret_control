#ifndef LIMIT_SWITCH_H_
#define LIMIT_SWITCH_H_

#include <chrono>
#include <functional>
#include <iostream>
#include <yaml-cpp/yaml.h>

class LimitSwitch {
 public:
  using StateChangeCallback = std::function<void(bool)>;
  
  // Constructor taking explicit parameters.
  LimitSwitch(int gpio_pin, int debounce_threshold_ms = 30);
  
  // Overloaded constructor that reads its configuration from a YAML node.
  explicit LimitSwitch(const YAML::Node &node);
  
  ~LimitSwitch();

  void HandleAlert(int level, uint32_t tick);
  bool IsPressed() const;
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
