#include "spiral_zipper.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

SpiralZipper::SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
                           int servo_enable_pin, int zipper_enc_cs_pin, int zipper_enc_clk_pin,
                           int zipper_enc_do_pin, int limit_switch_pin,
                           double extension_per_step, int debounce_threshold_ms)
    : servo_motor_(servo_pwm_pin, servo_dir_pin, servo_enc_a, servo_enc_b, servo_enable_pin),
      zipper_encoder_(zipper_enc_cs_pin, zipper_enc_clk_pin, zipper_enc_do_pin, 1023, 0),
      limit_switch_(limit_switch_pin, debounce_threshold_ms),
      extension_per_step_(extension_per_step),
      kp_(0.15) {  // Adjust kp_ as needed.
  // Configure the limit switch pull-up resistor.
  gpioSetPullUpDown(limit_switch_pin, PI_PUD_UP);
  // Ensure the motor is stopped initially.
  servo_motor_.stop();
}

SpiralZipper::~SpiralZipper() {
  servo_motor_.stop();
}

void SpiralZipper::Zero() {
  // Define a constant retraction velocity (in rad/s) for zeroing.
  const double zeroing_velocity = -0.5;
  servo_motor_.setTargetVelocity(zeroing_velocity);

  // Run until the limit switch is triggered.
  while (!limit_switch_.IsPressed()) {
    servo_motor_.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Stop the motor once zeroing is complete.
  servo_motor_.stop();
  // Reset the zipper encoder count.
  zipper_encoder_.ResetCount();
}

void SpiralZipper::ActuateLength(float goal_dist) {
  // Convert the desired extension (meters) into a target encoder count.
  int goal_count = static_cast<int>(goal_dist / extension_per_step_);
  zipper_encoder_.Update();
  int current_count = zipper_encoder_.GetCount();

  int error_counts = goal_count - current_count;
  // Use a proportional controller to convert position error (in counts) to a target velocity (rad/s).
  double target_velocity = kp_ * error_counts;

  // If the limit switch is pressed and we've extended past the target, stop the motor.
  if (limit_switch_.IsPressed() && current_count > goal_count) {
    servo_motor_.stop();
  } else {
    servo_motor_.setTargetVelocity(target_velocity);
  }
}

void SpiralZipper::Update() {
  // Update both the servo motor control and the zipper encoder.
  servo_motor_.update();
  zipper_encoder_.Update();
}

float SpiralZipper::GetExtension() const {
  // Return the current extension in meters.
  return zipper_encoder_.GetCount() * extension_per_step_;
}
