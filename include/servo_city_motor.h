#ifndef SERVO_CITY_MOTOR_H
#define SERVO_CITY_MOTOR_H

#include <pigpio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>

class ServoCityMotor {
 public:
  ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b, int enable_pin);
  ~ServoCityMotor();

  // Sets the target velocity in radians per second.
  void setTargetVelocity(double target_rad_per_sec);
  double getCurrentVelocity() const;
  // Must be called periodically (e.g., in a control loop) to update the control output.
  void update();
  void stop();

  // Debug variables.
  std::atomic<int> encoder_count{0};
  std::atomic<int> raw_a{0};
  std::atomic<int> raw_b{0};
  bool debug_encoder = true;

 private:
  const int pwm_pin_;
  const int dir_pin_;
  const int enc_a_;
  const int enc_b_;
  const int enable_pin_;

  // Motor specifications.
  const double max_rpm_ = 100.0;
  const double gear_ratio_ = 188.0;
  const double counts_per_rev_ = 5281.1;

  // Motion control.
  std::atomic<double> current_velocity_{0.0};
  double target_velocity_ = 0.0;

  // PID Control (tuning constants; may require adjustment).
  const double Kp_ = 0.3;
  const double Ki_ = 0.5;
  const double Kd_ = 0.02;
  double integral_ = 0.0;
  double prev_error_ = 0.0;

  // Output filtering.
  double output_filter_ = 0.0;
  const double filter_gain_ = 0.2;
  double velocity_filter_ = 0.0;
  const double velocity_filter_gain_ = 0.1;

  // Timing.
  std::chrono::time_point<std::chrono::steady_clock> last_update_;
  std::chrono::time_point<std::chrono::steady_clock> last_encoder_time_;

  // Encoder ISR handling.
  static ServoCityMotor* instance;
  static void encoderISR(int gpio, int level, uint32_t tick);
  void updateEncoder(int a, int b);
  double countsToRadians(int counts) const;
};

#endif  // SERVO_CITY_MOTOR_H
