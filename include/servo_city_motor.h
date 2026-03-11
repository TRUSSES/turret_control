#ifndef MOTOR_H
#define MOTOR_H

#include <pigpio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief ServoCityMotor controls the servocity brushed DC motor with encoder feedback.
 * 
 * This implementation uses sign-magnitude control: the PWM duty cycle (scaled to 0–255)
 * controls the speed, and the DIR pin is set according to the sign of the control output.
 */
class ServoCityMotor {
 public:
  ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b);
  
  /**
   * @brief Constructs a ServoCityMotor using configuration from a YAML node.
   * Expects keys: "pwm", "dir", "enca", "encb", and optional "pid" section.
   */
  explicit ServoCityMotor(const YAML::Node &node);
  
  ~ServoCityMotor();

  void setTargetVelocity(double target_rad_per_sec);
  double getTargetVelocity();
  double getCurrentVelocity() const;
  void update();
  void stop();
  
  // PID tuning methods for load optimization
  void setPIDGains(double kp, double ki, double kd) { Kp_ = kp; Ki_ = ki; Kd_ = kd; }
  void getPIDGains(double& kp, double& ki, double& kd) { kp = Kp_; ki = Ki_; kd = Kd_; }
  void resetPID() { integral_ = 0.0; prev_error_ = 0.0; }

  /// Set the PWM output directly.
  void setMotorOutput(int pwm);

  // Debug variables:
  std::atomic<int> encoder_count{0};
  std::atomic<int> raw_a{0};
  std::atomic<int> raw_b{0};
  bool debug_encoder = false;

 private:
  const int pwm_pin_;
  const int dir_pin_;
  const int enc_a_;
  const int enc_b_;
  
  // Motor parameters (for motor with gear ratio 188:1) - ACTIVE
  const double max_rpm_ = 50.0;  // Adjusted for 12V operation (24V rated = 100 RPM)
  const double gear_ratio_ = 188.0;
  const double counts_per_rev_ = 5281.1;

  // Motor parameters (for motor with gear ratio 99.5:1) - COMMENTED OUT
  // const double max_rpm_ = 100.0;  // Adjusted for 12V operation (24V rated = 180 RPM)
  // const double gear_ratio_ = 99.5;
  // const double counts_per_rev_ = 2786.2;

  std::atomic<double> current_velocity_{0.0};
  double target_velocity_ = 0.0;

  // PID control parameters - tunable for wheel load conditions
  // double Kp_ = 0.8;   // Proportional gain - increase for faster response, decrease if oscillating
  // double Ki_ = 0.1;   // Integral gain - increase to eliminate steady-state error
  // double Kd_ = 0.01;  // Derivative gain - increase to reduce overshoot, decrease if noisy

  double Kp_ = 0.0;   // Proportional gain - increase for faster response, decrease if oscillating
  double Ki_ = 0.0;   // Integral gain - increase to eliminate steady-state error
  double Kd_ = 0.0;  // Derivative gain - increase to reduce overshoot, decrease if noisy

  double integral_ = 0.0;
  double prev_error_ = 0.0;

  double output_filter_ = 0.0;
  const double filter_gain_ = 0.2;
  double velocity_filter_ = 0.0;
  const double velocity_filter_gain_ = 0.1;

  std::chrono::time_point<std::chrono::steady_clock> last_update_;
  std::chrono::time_point<std::chrono::steady_clock> last_encoder_time_;

  static ServoCityMotor* instance_[4];  // Support up to 4 motors
  static int instance_count_;
  int instance_id_;
  
  static void encoderISR(int gpio, int level, uint32_t tick);
  void updateEncoder(int a, int b);
  double countsToRadians(int counts) const;
};

#endif // MOTOR_H
