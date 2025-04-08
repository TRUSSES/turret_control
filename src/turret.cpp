#include "turret.h"
#include <cmath>
#include <iostream>
#include <thread>

static const double kTurretAngleScale = 2 * M_PI / 1024.0;

Turret::Turret(int socket, float x_offset, float y_offset)
    : turret_encoder_(0, 6, 5, 1023, 0),
      turret_limit_switch_(20, 30),
      pitch_motor_(0xA, socket),
      yaw_motor_(0xB, socket),
      spiral_zipper_(22, 27, 24, 25, 4, 13, 26, 19, 16, 0.000004453125, 30),
      x_offset_(x_offset),
      y_offset_(y_offset),
      prev_turret_angle_(0.0),
      last_turret_time_(std::chrono::steady_clock::now()) {
}

Turret::Turret(const YAML::Node &node)
    : turret_encoder_(node["turret_encoder"]),
      turret_limit_switch_(node["turret_limit_switch"]),
      pitch_motor_(node["pitch_motor"]["motor_id"].as<int>(), node["socket"].as<int>()),
      yaw_motor_(node["yaw_motor"]["motor_id"].as<int>(), node["socket"].as<int>()),
      spiral_zipper_(node["spiral_zipper"]),
      x_offset_(node["x_offset"].as<float>()),
      y_offset_(node["y_offset"].as<float>()),
      prev_turret_angle_(0.0),
      last_turret_time_(std::chrono::steady_clock::now()) {
}

Turret::~Turret() {
  pitch_motor_.zeroMotor();
}

void Turret::Init() {
  turret_encoder_.ResetCount();
  prev_turret_angle_ = GetTurretAngle();
  last_turret_time_ = std::chrono::steady_clock::now();
}

double Turret::GetTurretAngle() const {
  int count = turret_encoder_.GetCount();
  return count * kTurretAngleScale;
}

void Turret::Update() {
  turret_encoder_.Update();
  spiral_zipper_.UpdateEncoder();
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_turret_time_).count();
  if(dt > 0.01) {
    double current_angle = GetTurretAngle();
    prev_turret_angle_ = current_angle;
    last_turret_time_ = now;
  }
}

bool Turret::ActuateTurretCable(float goal_dist, float desired_pitch_deg, float /*yaw*/) {
  double desired_pitch = desired_pitch_deg * M_PI / 180.0;
  turret_encoder_.Update();
  spiral_zipper_.UpdateEncoder();

  // Calculate spiral zipper extension from its encoder.
  double L = spiral_zipper_.GetEncoderCount() * spiral_zipper_.GetExtensionPerStep();
  double turret_angle = GetTurretAngle();
  double current_r = x_offset_ + L * cos(turret_angle);
  double current_theta = atan((y_offset_ - L * sin(turret_angle)) / (x_offset_ + L * cos(turret_angle)));
  double current_cable_length = current_r * current_theta;
  double desired_r = x_offset_ + goal_dist * cos(desired_pitch);
  double desired_theta = atan((y_offset_ - goal_dist * sin(desired_pitch)) / (x_offset_ + goal_dist * cos(desired_pitch)));
  double desired_cable_length = desired_r * desired_theta;
  double error = desired_cable_length - current_cable_length;
  double v_target = cable_ff_ * desired_cable_length + cable_kp_ * error;
  
  double effective_L = (L > 0.001) ? L : 0.001;
  double desired_omega = v_target / effective_L;
  if(desired_omega > max_omega_rad_) {
    desired_omega = max_omega_rad_;
  } else if(desired_omega < -max_omega_rad_) {
    desired_omega = -max_omega_rad_;
  }
  
  const double cable_tolerance = 0.005;
  if(fabs(error) < cable_tolerance) {
    desired_omega = 0.0;
  }
  
  double desired_omega_deg = desired_omega * 180.0 / M_PI;
  std::cout << "Target pitch motor velocity = " << desired_omega_deg << " deg/s" << std::endl;
  pitch_motor_.sendCommandMITMode(0, desired_omega_deg, 0, 1.5, 0);
  spiral_zipper_.ActuateLength(goal_dist);
  
  bool zipper_reached = (fabs(goal_dist - L) < 0.001);
  bool cable_reached = (fabs(error) < cable_tolerance);
  return (zipper_reached && cable_reached);
}
