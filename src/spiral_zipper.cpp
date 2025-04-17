#include "spiral_zipper.h"
#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

SpiralZipper::SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
                           int zipper_enc_cs_pin, int zipper_enc_clk_pin,
                           int zipper_enc_do_pin, int limit_switch_pin, double extension_per_step,
                           int debounce_threshold_ms)
    : servo_motor_(servo_pwm_pin, servo_dir_pin, servo_enc_a, servo_enc_b),
      zipper_encoder_(zipper_enc_cs_pin, zipper_enc_clk_pin, zipper_enc_do_pin, 1023, 0),
      limit_switch_(limit_switch_pin, debounce_threshold_ms),
      extension_per_step_(extension_per_step),
      kp_(0.15),
      direction_(0) {
  // Additional initialization can be done here if needed.
}

SpiralZipper::SpiralZipper(const YAML::Node &config)
    : SpiralZipper(config["servo_pwm_pin"].as<int>(),
                   config["servo_dir_pin"].as<int>(),
                   config["servo_enc_a"].as<int>(),
                   config["servo_enc_b"].as<int>(),
                   config["zipper_enc_cs_pin"].as<int>(),
                   config["zipper_enc_clk_pin"].as<int>(),
                   config["zipper_enc_do_pin"].as<int>(),
                   config["limit_switch_pin"].as<int>(),
                   config["extension_per_step"].as<double>(),
                   config["debounce_threshold_ms"].as<int>()) {
}

SpiralZipper::~SpiralZipper() {
  // Optionally: stop the motor.
}

void SpiralZipper::Zero() {
  // For zeroing, we want to slowly retract the actuator.
  // Use a low negative target velocity for this purpose.
  const double retract_velocity = -0.3;  // rad/s (adjust for your system)
  servo_motor_.setTargetVelocity(retract_velocity);

  // Run the control loop until the limit switch signals that zero has been reached.
  while (!limit_switch_.IsPressed()) {
    servo_motor_.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Once the switch is triggered, stop the motor.
  servo_motor_.setTargetVelocity(0);
  // Allow the motor to settle.
  while (std::fabs(servo_motor_.getCurrentVelocity()) > 0.01) {
    servo_motor_.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  // Reset the encoder count after zeroing.
  zipper_encoder_.ResetCount();
}

void SpiralZipper::ActuateLength(float goal_dist) {
  // Convert goal distance (in meters) to encoder counts.
  int goal_count = static_cast<int>(goal_dist / extension_per_step_);
  
  // Update encoder count.
  zipper_encoder_.Update();
  int current_count = zipper_encoder_.GetCount();
  int error_counts = goal_count - current_count;
  
  // Compute a target velocity for the motor based on the positional error.
  // In sign-magnitude control, we use the new motor interface.
  double target_velocity = kp_ * error_counts;
  
  // If the limit switch is triggered and the actuator is beyond the goal,
  // stop the motor.
  if (limit_switch_.IsPressed() && current_count > goal_count) {
    servo_motor_.setTargetVelocity(0);
  } else {
    servo_motor_.setTargetVelocity(target_velocity);
  }
  
  // Let the control loop (update()) run externally.
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
