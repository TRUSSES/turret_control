#ifndef SPIRAL_ZIPPER_H_
#define SPIRAL_ZIPPER_H_

#include "servo_city_motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

class SpiralZipper {
 public:
  // Existing constructor (if needed for backward compatibility)
  SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
               int servo_enable_pin, int zipper_enc_cs_pin, int zipper_enc_clk_pin,
               int zipper_enc_do_pin, int limit_switch_pin, double extension_per_step,
               int debounce_threshold_ms);

  // New constructor that reads configuration from a YAML Node.
  explicit SpiralZipper(const YAML::Node &config);

  ~SpiralZipper();

  void Zero();
  void ActuateLength(float goal_dist);
  int GetEncoderCount() const;
  void UpdateEncoder();
  double GetExtensionPerStep() const;

 private:
  int ComputePWM(int goal_count, int current_count);
  void SetMotorOutput(int pwm_value);

  ServoCityMotor servo_motor_;
  Encoder zipper_encoder_;
  LimitSwitch limit_switch_;

  double extension_per_step_;
  double kp_;
  int direction_;
};

#endif  // SPIRAL_ZIPPER_H_
