#ifndef SPIRAL_ZIPPER_H_
#define SPIRAL_ZIPPER_H_

#include "servo_city_motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include <chrono>
#include <thread>

class SpiralZipper {
 public:
  // Constructs a SpiralZipper with:
  // - ServoCityMotor pins: pwm, dir, encoder A/B, and enable.
  // - Spiral zipper encoder pins: chip-select, clock, and data.
  // - Limit switch pin.
  // - extension_per_step converts encoder counts to meters.
  SpiralZipper(int servo_pwm_pin, int servo_dir_pin, int servo_enc_a, int servo_enc_b,
               int servo_enable_pin, int zipper_enc_cs_pin, int zipper_enc_clk_pin,
               int zipper_enc_do_pin, int limit_switch_pin,
               double extension_per_step = 0.000004453125, int debounce_threshold_ms = 30);

  ~SpiralZipper();

  // Zeroes the spiral zipper by commanding a fixed retraction velocity until the limit switch is pressed.
  void Zero();

  // Commands the spiral zipper to reach the desired extension (in meters) using a proportional position controller.
  void ActuateLength(float goal_dist);

  // Updates sensor readings; should be called periodically.
  void Update();

  // Returns the current extension in meters.
  float GetExtension() const;

 private:
  // The new servo motor controller.
  ServoCityMotor servo_motor_;
  // The dedicated encoder for the spiral zipper mechanism.
  Encoder zipper_encoder_;
  // The limit switch for detecting end-of-travel.
  LimitSwitch limit_switch_;

  // Conversion factor: meters per encoder count.
  double extension_per_step_;

  // Proportional gain for position control.
  double kp_;
};

#endif  // SPIRAL_ZIPPER_H_
