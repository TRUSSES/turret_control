#include "limit_switch.h"
#include <pigpio.h>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

// Define the static unordered_map to store instances by GPIO pin.
std::unordered_map<int, LimitSwitch*> LimitSwitch::instances_;

LimitSwitch::LimitSwitch(int gpio_pin, int debounce_threshold_ms, bool active_low)
    : gpio_pin_(gpio_pin),
      debounce_threshold_ms_(debounce_threshold_ms),
      active_low_(active_low),
      last_state_(false),
      pressed_(false),
      callback_(nullptr) {
  // Set the GPIO pin mode to input.
  gpioSetMode(gpio_pin_, PI_INPUT);

  // Configure pull-up/down based on expected active polarity.
  gpioSetPullUpDown(gpio_pin_, active_low_ ? PI_PUD_UP : PI_PUD_DOWN);

  // Initialize internal state from current pin reading so pressing before startup
  // is recognized on the first zeroing check.
  auto raw_level = gpioRead(gpio_pin_);
  bool is_pressed = active_low_ ? (raw_level == 0) : (raw_level != 0);
  last_state_ = is_pressed;
  pressed_ = last_state_;
  
  // Print a debug message.
  std::cout << "LimitSwitch on GPIO " << gpio_pin_
            << " (active_" << (active_low_ ? "low" : "high") << ") "
            << " initialized with pull-up enabled. Initial state: "
            << (last_state_ ? "PRESSED" : "RELEASED") << std::endl;

  // Prime the debounce timer: set it in the past so that the first press is not ignored.
  last_debounce_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(debounce_threshold_ms_);

  // Register this instance in the static map.
  instances_[gpio_pin_] = this;
  
  // Register the global callback with pigpio.
  int rc = gpioSetAlertFunc(gpio_pin_, LimitSwitch::GlobalAlertCallback);
  if (rc != 0) {
    throw std::runtime_error("Failed to set alert function for GPIO " + std::to_string(gpio_pin_));
  }
  std::cout << "Alert function set for LimitSwitch on GPIO " << gpio_pin_ << std::endl;
}



LimitSwitch::LimitSwitch(const YAML::Node &node)
    : LimitSwitch(node["limit_switch_pin"].as<int>(),
                  node["debounce_threshold_ms"] ? node["debounce_threshold_ms"].as<int>() : 30,
                  node["active_low"] ? node["active_low"].as<bool>() : true) {
}

LimitSwitch::~LimitSwitch() {
  // Remove the alert callback.
  gpioSetAlertFunc(gpio_pin_, nullptr);
  // Remove this instance from the map.
  instances_.erase(gpio_pin_);
}

void LimitSwitch::SetStateChangeCallback(StateChangeCallback callback) {
  callback_ = callback;
}

bool LimitSwitch::IsPressed() const {
  // Use direct read in case an alert/edge was missed so callers using polling
  // always get the latest switch state.
  int raw_level = gpioRead(gpio_pin_);
  return active_low_ ? (raw_level == 0) : (raw_level != 0);
}

void LimitSwitch::HandleAlert(int level, unsigned int tick) {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debounce_time_);
  if (elapsed.count() > debounce_threshold_ms_) {
    bool current_state = active_low_ ? (level == 0) : (level != 0);
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

void LimitSwitch::GlobalAlertCallback(int gpio, int level, unsigned int tick) {
  auto it = instances_.find(gpio);
  if (it != instances_.end() && it->second) {
    it->second->HandleAlert(level, tick);
  }
}
