#ifndef SERVO_CITY_MOTOR_H
#define SERVO_CITY_MOTOR_H

#include <pigpio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

class ServoCityMotor {
 public:
  ServoCityMotor(int pwm_pin, int dir_pin, int enc_a, int enc_b, int enable_pin);
  
  // Overloaded constructor that loads configuration from YAML.
  explicit ServoCityMotor(const YAML::Node &node);
  
  ~ServoCityMotor();

  void setTargetVelocity(double target_rad_per_sec);
  double getCurrentVelocity() const;
  void update();
  void stop();

  // New method to directly set motor output.
  void setMotorOutput(int pwm);

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

  const double max_rpm_ = 100.0;
  const double gear_ratio_ = 188.0;
  const double counts_per_rev_ = 5281.1;

  std::atomic<double> current_velocity_{0.0};
  double target_velocity_ = 0.0;

  const double Kp_ = 0.3;
  const double Ki_ = 0.5;
  const double Kd_ = 0.02;
  double integral_ = 0.0;
  double prev_error_ = 0.0;

  double output_filter_ = 0.0;
  const double filter_gain_ = 0.2;
  double velocity_filter_ = 0.0;
  const double velocity_filter_gain_ = 0.1;

  std::chrono::time_point<std::chrono::steady_clock> last_update_;
  std::chrono::time_point<std::chrono::steady_clock> last_encoder_time_;

  static ServoCityMotor* instance;
  static void encoderISR(int gpio, int level, uint32_t tick);
  void updateEncoder(int a, int b);
  double countsToRadians(int counts) const;
};

#endif  // SERVO_CITY_MOTOR_H
