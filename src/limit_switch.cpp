#include "limit_switch.h"
#include <pigpio.h>

LimitSwitch::LimitSwitch(int gpio_pin, int debounce_threshold_ms)
    : gpio_pin_(gpio_pin),
      debounce_threshold_ms_(debounce_threshold_ms),
      last_state_(true),
      pressed_(false),
      callback_(nullptr) {
  last_debounce_time_ = std::chrono::steady_clock::now();
}

LimitSwitch::LimitSwitch(const YAML::Node &node)
    : LimitSwitch(node["limit_switch_pin"].as<int>(),
                  node["debounce_threshold_ms"] ? node["debounce_threshold_ms"].as<int>() : 30) {
}

LimitSwitch::~LimitSwitch() {}

void LimitSwitch::SetStateChangeCallback(StateChangeCallback callback) {
  callback_ = callback;
}

void LimitSwitch::HandleAlert(int level, uint32_t tick) {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debounce_time_);
  if (duration.count() > debounce_threshold_ms_) {
    bool current_state = (level == 0);
    if (current_state != last_state_) {
      pressed_ = current_state;
      if (callback_) {
        callback_(pressed_);
      }
      std::cout << "LimitSwitch on GPIO " << gpio_pin_
                << (pressed_ ? " pressed" : " released") << std::endl;
      last_state_ = current_state;
    }
    last_debounce_time_ = now;
  }
}

bool LimitSwitch::IsPressed() const {
  return pressed_;
}
