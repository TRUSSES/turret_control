/*
 * @file turret.cpp
 * @author your name (you@domain.com)
 * @brief 
 * This file implements the Turret class, which controls the turret mechanism
 * of a robotic system. The turret consists of a pitch motor, yaw motor (not implemented), 
 * and a spiral zipper mechanism. The class provides methods to actuate the turret
 * cable, update the turret state, and get the current turret angle.
 * @version 0.1
 * @date 2025-04-07
 * 
 * 
 */

#include "turret.h"
#include <cmath>
#include <iostream>
#include <thread>

// Conversion constants for the turret encoder.
// Assume turret encoder: 1024 counts corresponds to 2*pi radians.
static const double kTurretAngleScale = 2 * M_PI / 1024.0;

/*
    @brief Constructs a Turret object.
    @details Initializes the turret encoder, limit switch, pitch motor, yaw motor,
             and spiral zipper. The encoder is used to track the turret's angle,
             while the limit switch prevents over-extension.
    @param socket The socket for the CubemarsControl instance.
    @param x_offset The x offset for the turret mechanism.
    @param y_offset The y offset for the turret mechanism.
*/
Turret::Turret(int socket, float x_offset, float y_offset)
    : turret_encoder_(0, 6, 5, 1023, 0),       // cs=0, clk=6, do=5, max=1023 for turret pitch
      turret_limit_switch_(20, 30),             // turret limit switch on pin 20, 30 ms debounce
      pitch_motor_(0xA, socket),                // pitch motor with motor id 0xA
      yaw_motor_(0xB, socket),                  // yaw motor with motor id 0xB (not implemented yet)
      // Create SpiralZipper using the new servocity motor.
      // For the spiral zipper, assign:
      //   servo motor: pwm pin = 22, dir pin = 27, encoder pins = 24 & 25, enable pin = 4
      //   zipper encoder: cs = 13, clk = 26, do = 19, limit switch pin = 16.
      spiral_zipper_(22, 27, 24, 25, 4, 13, 26, 19, 16,
                      0.000004453125, 30),
      x_offset_(x_offset),
      y_offset_(y_offset),
      prev_turret_angle_(0.0),
      last_turret_time_(std::chrono::steady_clock::now()) {
}

/*
    @brief Destructor for the Turret class.
    @details Stops the pitch motor and cleans up resources.
*/
Turret::~Turret() {
  // Optionally stop the pitch motor.
  pitch_motor_.zeroMotor();
}

/*
    @brief Initializes the turret components.
    @details Resets the turret encoder and initializes timing for angle estimation.
*/
void Turret::Init() {
  // Reset the turret encoder and initialize timing.
  turret_encoder_.ResetCount();
  prev_turret_angle_ = GetTurretAngle();
  last_turret_time_ = std::chrono::steady_clock::now();
}

double Turret::GetTurretAngle() const {
  // Convert turret encoder count to radians.
  int count = turret_encoder_.GetCount();
  return count * kTurretAngleScale;
}

void Turret::Update() {
  // Update turret encoder and spiral zipper.
  turret_encoder_.Update();
  spiral_zipper_.Update();

  // Update turret angular velocity estimation.
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_turret_time_).count();
  if (dt > 0.01) {
    double current_angle = GetTurretAngle();
    // (Velocity estimation could be used if needed.)
    prev_turret_angle_ = current_angle;
    last_turret_time_ = now;
  }
}

bool Turret::ActuateTurretCable(float goal_dist, float desired_pitch_deg, float /*yaw*/) {
  // Convert desired pitch from degrees to radians.
  double desired_pitch = desired_pitch_deg * M_PI / 180.0;

  // Update sensors.
  turret_encoder_.Update();
  spiral_zipper_.Update();

  // Get the current spiral zipper extension (in meters).
  double L = spiral_zipper_.GetExtension();

  // Get current turret pitch angle (in radians).
  double turret_angle = GetTurretAngle();

  // Compute current effective cable length using geometry.
  // Here, current_r and current_theta are computed as in the original code.
  double current_r = x_offset_ + L * cos(turret_angle);
  double current_theta = atan((y_offset_ - L * sin(turret_angle)) /
                              (x_offset_ + L * cos(turret_angle)));
  double current_cable_length = current_r * current_theta;

  // Compute desired cable length using goal_dist and desired pitch.
  double desired_r = x_offset_ + goal_dist * cos(desired_pitch);
  double desired_theta = atan((y_offset_ - goal_dist * sin(desired_pitch)) /
                              (x_offset_ + goal_dist * cos(desired_pitch)));
  double desired_cable_length = desired_r * desired_theta;

  // Calculate the cable length error.
  double error = desired_cable_length - current_cable_length;

  // Compute target linear cable velocity.
  double v_target = cable_ff_ * desired_cable_length + cable_kp_ * error;

  // For simplicity, assume the spiral zipper controller handles its own velocity.
  // Calculate desired turret angular velocity (rad/s) needed from the pitch motor.
  // Avoid division by zero by enforcing a minimum effective extension.
  double effective_L = (L > 0.001) ? L : 0.001;
  double desired_omega = v_target / effective_L;  // in rad/s

  // Saturate desired_omega.
  if (desired_omega > max_omega_rad_) {
    desired_omega = max_omega_rad_;
  } else if (desired_omega < -max_omega_rad_) {
    desired_omega = -max_omega_rad_;
  }

  // Apply a tolerance: if the cable length error is small, command zero velocity.
  const double cable_tolerance = 0.005;  // meters
  if (fabs(error) < cable_tolerance) {
    desired_omega = 0.0;
  }

  // Convert desired turret angular velocity to degrees per second for the pitch motor.
  double desired_omega_deg = desired_omega * 180.0 / M_PI;

  std::cout << "Turret Cable Control:" << std::endl
            << "  Current cable length = " << current_cable_length << " m" << std::endl
            << "  Desired cable length = " << desired_cable_length << " m" << std::endl
            << "  Error = " << error << " m" << std::endl
            << "  Target pitch motor velocity = " << desired_omega_deg << " deg/s" << std::endl;

  // Command the pitch motor via CubemarsControl.
  // sendCommandMITMode parameters: (pos, vel, kp, kd, torq).
  // Here, we use position = 0 and command the velocity.
  pitch_motor_.sendCommandMITMode(0, desired_omega_deg, 0, 1.5, 0);

  // Command the spiral zipper to actuate to the desired extension.
  spiral_zipper_.ActuateLength(goal_dist);

  // Check whether both the spiral zipper and cable (via pitch motor) have reached their targets.
  bool zipper_reached = (fabs(goal_dist - L) < 0.001);  // tolerance for extension
  bool cable_reached = (fabs(error) < cable_tolerance);

  return (zipper_reached && cable_reached);
}
