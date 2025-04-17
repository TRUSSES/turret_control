#ifndef SPIRAL_ZIPPER_H_
#define SPIRAL_ZIPPER_H_

#include "servo_city_motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

class SpiralZipper
{
public:
  // Primary constructor: all parameters are passed directly.
  SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
               int zipper_enc_cs_pin, int zipper_enc_clk_pin,
               int zipper_enc_do_pin, int limit_switch_pin, double extension_per_step,
               int debounce_threshold_ms);

  // YAML-based constructor.
  explicit SpiralZipper(const YAML::Node &config);

  ~SpiralZipper();

  // Zeroes the actuator by retracting until the limit switch is triggered.
  void Zero();

  // Actuates the spiral zipper to extend to a given goal distance.
  void ActuateLength(float goal_dist);

  // Returns the current encoder count.
  int GetEncoderCount() const;

  // Updates the encoder reading.
  void UpdateEncoder();

  // Returns the conversion factor (extension per encoder step).
  double GetExtensionPerStep() const;

  // In spiral_zipper.h, inside the SpiralZipper class public section
  double GetMotorVelocity() const { return servo_motor_.getCurrentVelocity(); }

private:
  // Use the new ServoCityMotor object (which implements sign-magnitude control).
  ServoCityMotor servo_motor_;

  // The encoder for the spiral zipper.
  Encoder zipper_encoder_;

  // The limit switch used for zeroing.
  LimitSwitch limit_switch_;

  // Conversion factor from encoder steps to physical extension.
  double extension_per_step_;

  // Proportional gain for converting positional error to desired velocity.
  double kp_;

  // Not needed for the new control but keeping track of direction if desired.
  int direction_;
};

#endif // SPIRAL_ZIPPER_H_
