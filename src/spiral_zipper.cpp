#include "spiral_zipper.h"
#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

SpiralZipper::SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
                           int zipper_enc_cs_pin, int zipper_enc_clk_pin,
                           int zipper_enc_do_pin, int limit_switch_pin, double extension_per_step,
                           int debounce_threshold_ms)
    : servo_motor_(servo_pwm_pin, servo_dir_pin, servo_enc_a, servo_enc_b),
      zipper_encoder_(zipper_enc_cs_pin, zipper_enc_clk_pin, zipper_enc_do_pin, 1023, 0),
      limit_switch_(limit_switch_pin, debounce_threshold_ms),
      extension_per_step_(extension_per_step),
      kp_(1.0),  // Higher gain for velocity control (vs 0.15 for PWM control)
      direction_(0),
      is_zeroing_(false),
      meters_per_enc_count_(extension_per_step * 4) {
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
  // Default velocity for zeroing
  Zero(-0.5);  // rad/s (adjust for your system)
}

void SpiralZipper::Zero(double retract_velocity) {
  // Set zeroing flag to prevent interference from ActuateLength
  is_zeroing_ = true;
  
  // First, stop any ongoing motion
  servo_motor_.setTargetVelocity(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give time to stop
  
  // For zeroing, we want to slowly retract the actuator.
  // Use the specified negative target velocity for this purpose.
  servo_motor_.setTargetVelocity(retract_velocity);
  
  std::cout << "Starting spiral zipper zeroing at velocity: " << retract_velocity << " rad/s" << std::endl;

  // Run the control loop until the limit switch signals that zero has been reached.
  while (!limit_switch_.IsPressed()) {
    // Update both motor and encoder during zeroing
    servo_motor_.update();
    zipper_encoder_.Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Limit switch pressed, stopping motor..." << std::endl;
  
  // Once the switch is triggered, stop the motor.
  servo_motor_.setTargetVelocity(0);
  // Allow the motor to settle.
  while (std::fabs(servo_motor_.getCurrentVelocity()) > 0.01) {
    servo_motor_.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  std::cout << "Motor stopped, resetting encoder count..." << std::endl;
  
  // Reset the encoder count after zeroing.
  zipper_encoder_.ResetCount();
  
  // Clear zeroing flag
  is_zeroing_ = false;
  
  std::cout << "Spiral zipper zeroing complete. Encoder count: " << zipper_encoder_.GetCount() << std::endl;
}

void SpiralZipper::ActuateLength(float goal_dist) {
  // Use the velocity-controlled version with a reasonable default max velocity
  ActuateLength(goal_dist, 2.0);  // Default 2.0 rad/s max velocity
}

void SpiralZipper::ActuateLength(float goal_dist, double max_velocity) {
  // Don't interfere if currently zeroing
  if (is_zeroing_) {
    return;
  }
  
  // Update encoder to get latest count
  zipper_encoder_.Update();
  
  // Convert the goal distance to the corresponding encoder count
  // Using same logic as reference: goal_dist / (EXTENSION_PER_STEP * 4)
  int goal_count = static_cast<int>(goal_dist / (extension_per_step_ * 4));
  
  // Get current count from encoder (this is the key fix!)
  int current_count = zipper_encoder_.GetCount();
  
  // Calculate error based on encoder count
  int error_counts = goal_count - current_count;
  
  // Log significant position changes or errors (commented out for performance)
  // static double last_logged_position = -999.0;  // Track last logged position
  // double current_length = current_count * (extension_per_step_ * 4);
  
  // Log when starting a new movement or significant position change
  // if (std::abs(current_length - last_logged_position) > 0.01) {  // Log every 1cm change
  //   std::cout << "SpiralZipper: Moving to " << goal_dist << "m (current: " << current_length << "m)" << std::endl;
  //   last_logged_position = current_length;
  // }
  
  // Velocity-based proportional control
  // Scale the gain appropriately for velocity control (higher than PWM-based)
  double target_velocity = kp_ * error_counts;
  
  // Add minimum velocity threshold for small errors to ensure movement
  if (std::abs(error_counts) > 5 && std::abs(target_velocity) < 0.1) {
    target_velocity = (error_counts > 0) ? 0.1 : -0.1;
  }
  
  // Apply velocity limiting while preserving direction
  if (std::abs(target_velocity) > max_velocity) {
    target_velocity = (error_counts > 0) ? max_velocity : -max_velocity;
  }
  
  // Safety checks
  bool should_stop = false;
  std::string stop_reason = "";
  
  // Check limit switch - stop if trying to retract past zero
  if (limit_switch_.IsPressed() && target_velocity < 0) {
    should_stop = true;
    stop_reason = "Limit switch pressed during retraction";
  }
  
  // Check if we've reached the target (within tolerance) - using smaller deadband for velocity control
  const int deadband = 50;  // Smaller deadband for velocity control precision
  if (std::abs(error_counts) <= deadband) {
    should_stop = true;
    stop_reason = "Target position reached (within deadband)";
  }
  
  if (should_stop) {
    servo_motor_.setTargetVelocity(0);
    // Log when target is reached or stopped (commented out for performance)
    // double current_length = current_count * (extension_per_step_ * 4);
    // std::cout << "SpiralZipper: " << stop_reason << " at " << current_length << "m" << std::endl;
  } else {
    servo_motor_.setTargetVelocity(target_velocity);
  }
  
  // Let the control loop (update()) run externally.
}

void SpiralZipper::Stop() {
  servo_motor_.setTargetVelocity(0.0);
}

int SpiralZipper::GetEncoderCount() const {
  return zipper_encoder_.GetCount();
}

void SpiralZipper::UpdateEncoder() {
  zipper_encoder_.Update();
}

void SpiralZipper::UpdateMotor() {
  // Don't interfere with motor updates during zeroing
  // The Zero() method handles motor updates internally
  if (!is_zeroing_) {
    servo_motor_.update();
  }
}

double SpiralZipper::GetExtensionPerStep() const {
  return extension_per_step_;
}

double SpiralZipper::GetExtension() const {
  // Use the same calculation as reference: count * (EXTENSION_PER_STEP * 4)
  return zipper_encoder_.GetCount() * (extension_per_step_ * 4);
}
