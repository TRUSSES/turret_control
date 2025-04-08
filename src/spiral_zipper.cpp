#include "spiral_zipper.h"
#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

SpiralZipper::SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
                           int servo_enable_pin, int zipper_enc_cs_pin, int zipper_enc_clk_pin,
                           int zipper_enc_do_pin, int limit_switch_pin, double extension_per_step,
                           int debounce_threshold_ms)
    : servo_motor_(servo_pwm_pin, servo_dir_pin, servo_enc_a, servo_enc_b, servo_enable_pin),
      zipper_encoder_(zipper_enc_cs_pin, zipper_enc_clk_pin, zipper_enc_do_pin, 1023, 0),
      limit_switch_(limit_switch_pin, debounce_threshold_ms),
      extension_per_step_(extension_per_step),
      kp_(0.15),
      direction_(0) {
}

SpiralZipper::SpiralZipper(const YAML::Node &config)
    : SpiralZipper(config["servo_pwm_pin"].as<int>(),
                   config["servo_dir_pin"].as<int>(),
                   config["servo_enc_a"].as<int>(),
                   config["servo_enc_b"].as<int>(),
                   config["servo_enable_pin"].as<int>(),
                   config["zipper_enc_cs_pin"].as<int>(),
                   config["zipper_enc_clk_pin"].as<int>(),
                   config["zipper_enc_do_pin"].as<int>(),
                   config["limit_switch_pin"].as<int>(),
                   config["extension_per_step"].as<double>(),
                   config["debounce_threshold_ms"].as<int>()) {
}

SpiralZipper::~SpiralZipper() {
  // Optionally, stop the motor here.
}

void SpiralZipper::Zero() {
  // Retract slowly until the limit switch is triggered.
  while (!limit_switch_.IsPressed()) {
    SetMotorOutput(1650); // Example PWM value to retract slowly.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  SetMotorOutput(1500); // Stop the motor.
  zipper_encoder_.ResetCount();
}

void SpiralZipper::ActuateLength(float goal_dist) {
  int goal_count = static_cast<int>(goal_dist / extension_per_step_);
  zipper_encoder_.Update();
  int current_count = zipper_encoder_.GetCount();
  int error_counts = goal_count - current_count;
  double target_velocity = kp_ * error_counts;

  if (limit_switch_.IsPressed() && current_count > goal_count) {
    SetMotorOutput(1500);
  } else {
    int pwm_value = ComputePWM(goal_count, current_count);
    SetMotorOutput(pwm_value);
  }
}

int SpiralZipper::ComputePWM(int goal_count, int current_count) {
  int error = goal_count - current_count;
  int pwm_calculated = static_cast<int>(error * kp_ + 1500);
  const int deadband = 200;
  int control_action = 1500;
  
  if (error > deadband / 2) {
    control_action = std::max(1350, std::min(1000, pwm_calculated));  // Example values.
    direction_ = -1;
  } else if (error < -deadband / 2) {
    control_action = std::max(1650, std::min(2000, pwm_calculated));
    direction_ = 1;
  } else {
    if (direction_ == -1) {
      if (error > 0) {
        control_action = std::max(1350, std::min(1000, pwm_calculated));
      } else {
        direction_ = 0;
      }
    } else if (direction_ == 1) {
      if (error < 0) {
        control_action = std::max(1650, std::min(2000, pwm_calculated));
      } else {
        direction_ = 0;
      }
    }
    if (direction_ == 0) {
      control_action = 1500;
    }
  }
  return control_action;
}

void SpiralZipper::SetMotorOutput(int pwm_value) {
  servo_motor_.setMotorOutput(pwm_value);
}

int SpiralZipper::GetEncoderCount() const {
  return zipper_encoder_.GetCount();
}

void SpiralZipper::UpdateEncoder() {
  zipper_encoder_.Update();
}

double SpiralZipper::GetExtensionPerStep() const {
  return extension_per_step_;
}
