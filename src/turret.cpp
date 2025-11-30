#include "turret.h"
#include <cmath>
#include <iostream>
#include <thread>

static const double kTurretAngleScale = 2 * M_PI / 1024.0;

Turret::Turret(int socket, float x_offset, float y_offset)
    : turret_encoder_(0, 6, 5, 1023, 0),
      pitch_encoder_(0, 0, 0, 1023, 0),  // TODO: Set correct pins for pitch encoder
      turret_limit_switch_(20, 30),
      pitch_motor_(0xA, socket),
      yaw_motor_(0xB, socket),
      spiral_zipper_(22, 27, 24, 25, 4, 13, 19, 16, 0.0005725, 30),
      x_offset_(x_offset),
      y_offset_(y_offset),
      prev_turret_angle_(0.0),
      last_turret_time_(std::chrono::steady_clock::now()) {

    // Initialize Pi3Hat for spool motor
    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_config;
    pi3hat_config.can[4].slow_bitrate = 1000000;
    pi3hat_config.can[4].fdcan_frame = false;
    pi3hat_config.can[4].bitrate_switch = false;
    pi3hat_config.can[0].slow_bitrate = 0;
    pi3hat_config.can[1].slow_bitrate = 0;
    pi3hat_config.can[2].slow_bitrate = 0;
    pi3hat_config.can[3].slow_bitrate = 0;

    pi3hat_ = std::make_unique<mjbots::pi3hat::Pi3Hat>(pi3hat_config);
    spool_motor_ = std::make_unique<CubemarsPi3Hat>(10, 0, pi3hat_.get());
}

Turret::Turret(const YAML::Node &node)
    : turret_encoder_(node["turret_encoder"]),
      pitch_encoder_(node["pitch_encoder"]),
      turret_limit_switch_(node["turret_limit_switch"]),
      pitch_motor_(node["pitch_motor"]["motor_id"].as<int>(), node["socket"].as<int>()),
      yaw_motor_(node["yaw_motor"]["motor_id"].as<int>(), node["socket"].as<int>()),
      spiral_zipper_(node["spiral_zipper"]),
      x_offset_(node["x_offset"].as<float>()),
      y_offset_(node["y_offset"].as<float>()),
      prev_turret_angle_(0.0),
      last_turret_time_(std::chrono::steady_clock::now()) {

    // Initialize Pi3Hat for spool motor
    mjbots::pi3hat::Pi3Hat::Configuration pi3hat_config;
    pi3hat_config.can[4].slow_bitrate = 1000000;
    pi3hat_config.can[4].fdcan_frame = false;
    pi3hat_config.can[4].bitrate_switch = false;
    pi3hat_config.can[0].slow_bitrate = 0;
    pi3hat_config.can[1].slow_bitrate = 0;
    pi3hat_config.can[2].slow_bitrate = 0;
    pi3hat_config.can[3].slow_bitrate = 0;

    pi3hat_ = std::make_unique<mjbots::pi3hat::Pi3Hat>(pi3hat_config);

    int spool_motor_id = node["spool_motor"]["motor_id"] ? node["spool_motor"]["motor_id"].as<int>() : 10;
    spool_motor_ = std::make_unique<CubemarsPi3Hat>(spool_motor_id, 0, pi3hat_.get());
}

Turret::~Turret() {
  pitch_motor_.zeroMotor();
  if (spool_motor_) {
    spool_motor_->exitMITMode();
  }
}

void Turret::Init() {
  turret_encoder_.ResetCount();
  pitch_encoder_.ResetCount();
  prev_turret_angle_ = GetTurretAngle();
  last_turret_time_ = std::chrono::steady_clock::now();

  // Initialize spool motor
  if (spool_motor_) {
    spool_motor_->enterMITMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spool_motor_->zeroMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    spool_tracker_.initialize();
  }
}

double Turret::GetTurretAngle() const {
  int count = turret_encoder_.GetCount();
  return count * kTurretAngleScale;
}

void Turret::Update() {
  turret_encoder_.Update();
  pitch_encoder_.Update();
  spiral_zipper_.UpdateEncoder();
  spiral_zipper_.UpdateMotor();  // Critical: Update motor control loop
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

  // Get the current spiral zipper extension (in meters).
  double L = spiral_zipper_.GetExtension();
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

void Turret::ZeroSpiralZipper() {
  spiral_zipper_.Zero();
}

void Turret::ZeroSpiralZipper(double retract_velocity) {
  spiral_zipper_.Zero(retract_velocity);
}

void Turret::ActuateSpiralZipperLength(float goal_dist) {
  spiral_zipper_.ActuateLength(goal_dist);
}

void Turret::ActuateSpiralZipperLength(float goal_dist, double max_velocity) {
  spiral_zipper_.ActuateLength(goal_dist, max_velocity);
}

void Turret::StopSpiralZipper() {
  spiral_zipper_.Stop();
}

double Turret::GetSpiralZipperExtension() const {
  return spiral_zipper_.GetExtension();
}

double Turret::GetSpiralZipperVelocity() const {
  return spiral_zipper_.GetMotorVelocity();
}

int Turret::GetSpiralZipperEncoderCount() const {
  return spiral_zipper_.GetEncoderCount();
}

// Spool control methods
void Turret::ZeroSpool() {
  if (spool_motor_) {
    spool_motor_->zeroMotor();
    spool_tracker_.initialize();
  }
}

double Turret::GetSpoolWireLength() const {
  return spool_tracker_.getWireLength();
}

double Turret::GetPitchAngle() const {
  int count = pitch_encoder_.GetCount();
  return count * kPitchAngleScale;
}

void Turret::ActuateSpoolLength(float target_length_m, float rate_m_s) {
  if (!spool_motor_) return;

  float current_length = spool_tracker_.getWireLength();
  float length_change = target_length_m - current_length;

  if (std::abs(length_change) < 0.001f) {
    // Already at target
    spool_motor_->sendCommandMITMode(0.0f, 0.0f, 0.0f, 0.3f, 0.0f);
    return;
  }

  // Calculate velocity command
  float direction = (length_change > 0) ? 1.0f : -1.0f;
  float velocity_rad_s = (rate_m_s * direction) / SpoolTracker::SPOOL_RADIUS_M;

  // Send velocity command
  spool_motor_->sendCommandMITMode(0.0f, velocity_rad_s, 0.0f, 0.5f, 0.0f);

  // Update tracker with measured velocity
  float measured_vel = spool_motor_->getVelocity();
  spool_tracker_.update(measured_vel);
}

// Combined spiral zipper + spool control
void Turret::ActuateCoupledExtension(float goal_dist, float desired_pitch_deg) {
  // Update all encoders
  turret_encoder_.Update();
  pitch_encoder_.Update();
  spiral_zipper_.UpdateEncoder();

  // Get current state
  double L = spiral_zipper_.GetExtension();
  double current_pitch = GetPitchAngle();
  double current_wire = spool_tracker_.getWireLength();

  // Target state
  double desired_pitch_rad = desired_pitch_deg * M_PI / 180.0;

  // Calculate required wire length based on geometry
  // Wire connects from spool through pivot to end of spiral zipper
  // When spiral zipper extends, wire must also extend to maintain pitch angle
  //
  // Simple geometry: wire_length = L * sin(pitch_angle) + offset
  // The offset accounts for the base cable length when fully retracted
  double wire_for_current = L * std::sin(current_pitch);
  double wire_for_target = goal_dist * std::sin(desired_pitch_rad);

  // Calculate required wire extension rate based on spiral zipper extension rate
  // This couples the two actuators together
  double spiral_velocity = spiral_zipper_.GetMotorVelocity();
  double wire_rate_m_s = spiral_velocity * std::sin(desired_pitch_rad);

  // Calculate error for pitch angle control
  double pitch_error = desired_pitch_rad - current_pitch;

  // Proportional control for pitch correction via wire length
  // Positive pitch error -> need more wire extension
  double pitch_correction = cable_kp_ * pitch_error;

  // Total wire target includes geometry + pitch correction
  double total_wire_rate = wire_rate_m_s + pitch_correction;

  // Clamp wire rate
  if (total_wire_rate > 0.1) total_wire_rate = 0.1;
  if (total_wire_rate < -0.1) total_wire_rate = -0.1;

  // Send commands
  // Spiral zipper
  spiral_zipper_.ActuateLength(goal_dist);

  // Spool motor - velocity control with measured feedback
  if (spool_motor_) {
    float velocity_rad_s = static_cast<float>(total_wire_rate / SpoolTracker::SPOOL_RADIUS_M);
    spool_motor_->sendCommandMITMode(0.0f, velocity_rad_s, 0.0f, 0.5f, 0.0f);

    // Update tracker with measured velocity
    float measured_vel = spool_motor_->getVelocity();
    spool_tracker_.update(measured_vel);
  }

  // Debug output
  std::cout << "Coupled: L=" << L << "m, Pitch=" << (current_pitch * 180.0 / M_PI)
            << "deg, Wire=" << current_wire << "m, WireRate=" << total_wire_rate
            << "m/s" << std::endl;
}
