#include "turret.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

Turret::Turret(const YAML::Node &node)
    : turret_encoder_(node["turret_encoder"]),
      turret_limit_switch_(node["turret_limit_switch"]),
      spiral_zipper_(node["spiral_zipper"]),
      x_offset_(node["x_offset"].as<float>()),
      y_offset_(node["y_offset"].as<float>()),
      pitch_encoder_scale_rad_per_count_(
          2.0 * M_PI /
          (node["turret_encoder"]["counts_per_rev"] ? node["turret_encoder"]["counts_per_rev"].as<double>()
                                                     : ((node["turret_encoder"]["encoder_max_value"]
                                                             ? node["turret_encoder"]["encoder_max_value"].as<double>()
                                                             : 1023.0) +
                                                        1.0))),
      pitch_encoder_sign_(node["turret_encoder"]["sign"] ? node["turret_encoder"]["sign"].as<double>() : 1.0),
      pitch_kinematics_sign_(node["pitch_kinematics_sign"] ? node["pitch_kinematics_sign"].as<double>() : -1.0),
      pitch_encoder_zero_angle_rad_(0.0),
      pitch_angle_offset_rad_(node["pitch_angle_offset_deg"] ? (node["pitch_angle_offset_deg"].as<float>() * M_PI / 180.0)
                                                           : 0.0),
      pitch_motor_length_zero_angle_rad_(0.0),
      pitch_motor_cable_zero_length_m_(0.0) {
  // Initialize Pi3Hat for all Cubemars motors
  mjbots::pi3hat::Pi3Hat::Configuration pi3hat_config;
  pi3hat_config.can[4].slow_bitrate = 1000000;
  pi3hat_config.can[4].fdcan_frame = false;
  pi3hat_config.can[4].bitrate_switch = false;
  pi3hat_config.can[0].slow_bitrate = 0;
  pi3hat_config.can[1].slow_bitrate = 0;
  pi3hat_config.can[2].slow_bitrate = 0;
  pi3hat_config.can[3].slow_bitrate = 0;

  pi3hat_ = std::make_unique<mjbots::pi3hat::Pi3Hat>(pi3hat_config);

  // Initialize motors via Pi3Hat from config (motor_id, can_bus, pi3hat_ptr)
  int pitch_motor_id = node["pitch_motor"]["motor_id"] ? node["pitch_motor"]["motor_id"].as<int>() : 10;
  int yaw_motor_id = node["yaw_motor"]["motor_id"] ? node["yaw_motor"]["motor_id"].as<int>() : 11;

  pitch_motor_ = std::make_unique<CubemarsPi3Hat>(pitch_motor_id, 0, pi3hat_.get());
  yaw_motor_ = std::make_unique<CubemarsPi3Hat>(yaw_motor_id, 0, pi3hat_.get());
  std::cout << "DEBUG: Pitch motor initialized with ID " << pitch_motor_id << std::endl;
  std::cout << "DEBUG: Yaw motor initialized with ID " << yaw_motor_id << std::endl;
  std::cout << "DEBUG: Pitch encoder scale = "
            << (pitch_encoder_scale_rad_per_count_ * 180.0 / M_PI)
            << " deg/count, sign = " << pitch_encoder_sign_
            << ", kinematics_sign = " << pitch_kinematics_sign_
            << ", zero-angle offset = " << (pitch_angle_offset_rad_ * 180.0 / M_PI)
            << " deg" << std::endl;
}

Turret::~Turret() {
  if (pitch_motor_) {
    pitch_motor_->exitMITMode();
  }
  if (yaw_motor_) {
    yaw_motor_->exitMITMode();
  }
}

void Turret::Init() {
  turret_encoder_.ResetCount();
}

double Turret::GetRawTurretEncoderAngle() const {
  return turret_encoder_.GetCount() * pitch_encoder_sign_ * pitch_encoder_scale_rad_per_count_;
}

double Turret::ToKinematicPitchAngle(double pitch_angle_rad) const {
  return pitch_kinematics_sign_ * pitch_angle_rad;
}

void Turret::Update() {
  turret_encoder_.Update();
  spiral_zipper_.UpdateEncoder();
  spiral_zipper_.UpdateMotor();  // Critical: Update motor control loop
}

double Turret::ComputeCableLength(float extension, double pitch_angle_rad) const {
  // Geometry from docs: b = sqrt((x + a cosθ)^2 + (y + a sinθ)^2),
  // where y offset is expected to be (y - h).
  double dx = x_offset_ + extension * std::cos(pitch_angle_rad);
  double dy = y_offset_ + extension * std::sin(pitch_angle_rad);
  return std::sqrt((dx * dx) + (dy * dy));
}

void Turret::ComputeCableLengthJacobian(float extension,
                                        double pitch_angle_rad,
                                        double &dc_dz,
                                        double &dc_dtheta) const {
  const double dx = x_offset_ + extension * std::cos(pitch_angle_rad);
  const double dy = y_offset_ + extension * std::sin(pitch_angle_rad);
  const double c = ComputeCableLength(extension, pitch_angle_rad);
  if (c <= 1e-9) {
    dc_dz = 0.0;
    dc_dtheta = 0.0;
    return;
  }

  dc_dz = (dx * std::cos(pitch_angle_rad) + dy * std::sin(pitch_angle_rad)) / c;
  dc_dtheta = (extension * (-dx * std::sin(pitch_angle_rad) + dy * std::cos(pitch_angle_rad))) / c;
}

double Turret::GetPitchCableLength() const {
  if (!pitch_motor_) {
    return 0.0;
  }
  return pitch_motor_cable_zero_length_m_ +
         (pitch_motor_->getPosition() - pitch_motor_length_zero_angle_rad_) * pitch_motor_radius_m_;
}

bool Turret::ActuateTurretCable(float goal_dist, float desired_pitch_rad, float max_zipper_velocity,
                                bool hold_pitch) {
  if (!pitch_motor_) {
    std::cerr << "ActuateTurretCable aborted: pitch motor unavailable" << std::endl;
    return false;
  }

  turret_encoder_.Update();
  spiral_zipper_.UpdateEncoder();
  double current_pitch_angle = GetPitchAngle();

  double zipper_extension = spiral_zipper_.GetExtension();
  const bool sz_limit_pressed = spiral_zipper_.IsLimitSwitchPressed();
  static bool last_sz_limit_pressed = false;
  if (sz_limit_pressed) {
    if (!last_sz_limit_pressed) {
      std::cout << "ActuateTurretCable: Spiral zipper limit switch active -> stopping zipper motor." << std::endl;
    }
    spiral_zipper_.Stop();
  }
  last_sz_limit_pressed = sz_limit_pressed;

  const bool turret_limit_pressed = turret_limit_switch_.IsPressed();
  static bool last_turret_limit_pressed = false;
  double current_cable_length = GetPitchCableLength();
  double desired_cable_length = ComputeCableLength(goal_dist, ToKinematicPitchAngle(desired_pitch_rad));
  double cable_error = desired_cable_length - current_cable_length;
  double zipper_error = goal_dist - zipper_extension;
  double pitch_error = desired_pitch_rad - current_pitch_angle;

  // Track pitch angle and compute Jacobian terms at the current pose.
  double dc_dz = 0.0;
  double dc_dpitch = 0.0;
  ComputeCableLengthJacobian(spiral_zipper_.GetExtension(),
                             ToKinematicPitchAngle(current_pitch_angle),
                             dc_dz,
                             dc_dpitch);
  dc_dpitch *= pitch_kinematics_sign_;

  constexpr double kMinCommandVelocity = 0.03;
  constexpr double kMaxCommandVelocity = 2.0;
  double zipper_velocity = std::abs(max_zipper_velocity);
  if (zipper_velocity <= 0.0) {
    zipper_velocity = kMaxCommandVelocity;
  } else {
    zipper_velocity = std::clamp(zipper_velocity, kMinCommandVelocity, kMaxCommandVelocity);
  }

  // Extension command -> desired zipper speed (m/s), then map to coupled cable dynamics.
  double zipper_velocity_ref = std::clamp(
      cable_kp_ * zipper_error,
      -zipper_velocity,
      zipper_velocity);
  if (std::fabs(zipper_error) <= 0.001) {
    zipper_velocity_ref = 0.0;
  } else if (std::fabs(zipper_velocity_ref) < kMinCommandVelocity) {
    zipper_velocity_ref = (zipper_velocity_ref >= 0.0) ? kMinCommandVelocity : -kMinCommandVelocity;
  }

  if (!sz_limit_pressed) {
    spiral_zipper_.ActuateLength(goal_dist, zipper_velocity);
  }

  // Velocity-level coupled control using the Jacobian:
  //   dc/dt = dc/dz * dz/dt + dc/dtheta * dtheta/dt
  constexpr double kCouplingDeadband = 0.002;
  constexpr double kPitchHoldDeadband = 0.008726646259971648;  // 0.5 deg
  constexpr double kSmallPitchCmd = 0.005;
  double desired_cable_velocity = std::clamp(
      cable_kp_ * cable_error,
      -max_zipper_rate_coupled_,
      max_zipper_rate_coupled_);

  double coupling_pitch_cmd = 0.0;
  const double zipper_error_reached = std::fabs(zipper_error) <= kCouplingDeadband;
  const double cable_error_reached = std::fabs(cable_error) <= 0.005;
  if (!turret_limit_pressed && std::fabs(desired_cable_velocity) > kSmallPitchCmd) {
    const double coupling_numerator = desired_cable_velocity - dc_dz * zipper_velocity_ref;
    const double jacobian_scale = dc_dpitch * dc_dpitch + jacobian_damping_ * jacobian_damping_;
    if (jacobian_scale > 0.0) {
      coupling_pitch_cmd = pitch_coupling_gain_ * coupling_numerator * (dc_dpitch / jacobian_scale);
    }
  }

  double pitch_feedback_cmd = 0.0;
  if (std::fabs(pitch_error) > kPitchHoldDeadband) {
    pitch_feedback_cmd = pitch_kp_ * pitch_error;
  }

  double desired_pitch_motor_velocity = std::clamp(
      coupling_pitch_cmd + pitch_feedback_cmd,
      -max_omega_rad_,
      max_omega_rad_);

  // The pitch limit switch is only a stop in the direction that drove into it during zeroing.
  if (turret_limit_pressed && desired_pitch_motor_velocity < 0.0) {
    desired_pitch_motor_velocity = 0.0;
    if (!last_turret_limit_pressed) {
      std::cout << "ActuateTurretCable: Turret limit switch active -> blocking negative pitch motor motion."
                << std::endl;
    }
  }
  last_turret_limit_pressed = turret_limit_pressed;

  if (cable_error_reached && std::fabs(zipper_error) <= 0.001 && std::fabs(desired_pitch_motor_velocity) < kSmallPitchCmd) {
    desired_pitch_motor_velocity = 0.0;
  }

  desired_pitch_motor_velocity = std::clamp(desired_pitch_motor_velocity,
                                            -max_omega_rad_,
                                            max_omega_rad_);
  pitch_motor_->sendCommandMITMode(0.0, desired_pitch_motor_velocity, 0.0, 1.5, 0.0);

  bool pitch_reached = std::fabs(pitch_error) < 0.03; // ~1.7 deg
  bool zipper_reached = (std::fabs(goal_dist - zipper_extension) < 0.001);
  bool cable_reached = (std::fabs(cable_error) < 0.005);
  if (!zipper_reached || !pitch_reached || !cable_reached) {
    static int active_log_counter = 0;
    ++active_log_counter;
    if (active_log_counter % 100 == 0) {
      std::cout << "ActuateTurretCable: pitch="
                << (current_pitch_angle * 180.0 / M_PI) << " deg, target="
                << (desired_pitch_rad * 180.0 / M_PI) << " deg, error="
                << (pitch_error * 180.0 / M_PI) << " deg, pitch_cmd="
                << (desired_pitch_motor_velocity * 180.0 / M_PI) << " deg/s, ext="
                << zipper_extension << " m, goal_ext=" << goal_dist << " m" << std::endl;
    }
    return false;
  }

  pitch_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);
  return true;
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

bool Turret::ZeroTurret(double retract_velocity, double pitch_velocity_ratio) {
  constexpr double kMinZeroSpeed = 0.005;
  constexpr double kMaxZeroSpeed = 3.0;
  constexpr double kDefaultPitchVelocityRatio = 1.0;

  if (!pitch_motor_) {
    std::cout << "ZeroTurret aborted: pitch motor unavailable" << std::endl;
    return false;
  }

  if (retract_velocity > 0.0) {
    retract_velocity = -retract_velocity;
  }
  if (std::abs(retract_velocity) < kMinZeroSpeed) {
    retract_velocity = -kMinZeroSpeed;
  } else if (std::abs(retract_velocity) > kMaxZeroSpeed) {
    retract_velocity = -kMaxZeroSpeed;
  }

  double ratio = pitch_velocity_ratio;
  if (ratio <= 0.0) {
    ratio = kDefaultPitchVelocityRatio;
  }

  double pitch_velocity = retract_velocity * ratio;
  if (std::abs(pitch_velocity) < kMinZeroSpeed) {
    pitch_velocity = (retract_velocity >= 0.0) ? kMinZeroSpeed : -kMinZeroSpeed;
  } else if (std::abs(pitch_velocity) > kMaxZeroSpeed) {
    pitch_velocity = (pitch_velocity > 0.0) ? kMaxZeroSpeed : -kMaxZeroSpeed;
  }

  spiral_zipper_.Stop();
  pitch_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);
  pitch_motor_->enterMITMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "ZeroTurret: pitch motor initialized (MIT mode entered), idling at pos="
            << pitch_motor_->getPosition() << ", vel=" << pitch_motor_->getVelocity() << std::endl;

  bool sz_zeroed = false;
  bool pitch_zeroed = false;
  int loop_counter = 0;
  int pitch_sign_retries = 0;
  bool pitch_sign_flipped = false;
  bool waiting_for_sz_after_pitch_limit = false;

  while (!sz_zeroed || !pitch_zeroed) {
    ++loop_counter;
    float current_pitch_vel = pitch_motor_->getVelocity();
    float current_pitch_pos = pitch_motor_->getPosition();
    bool turret_limit_pressed = turret_limit_switch_.IsPressed();

    if (!sz_zeroed) {
      if (spiral_zipper_.IsLimitSwitchPressed()) {
        spiral_zipper_.Stop();
        sz_zeroed = true;
        spiral_zipper_.ResetCount();
        std::cout << "SZ zero switch pressed: stopping zipper and resetting encoder" << std::endl;
      } else {
        spiral_zipper_.SetMotorVelocity(retract_velocity);
      }
    }

    if (!pitch_zeroed) {
      if (turret_limit_pressed) {
        pitch_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);
        if (sz_zeroed) {
          pitch_encoder_zero_angle_rad_ = GetRawTurretEncoderAngle();
          pitch_motor_length_zero_angle_rad_ = pitch_motor_->getPosition();
          pitch_motor_cable_zero_length_m_ = ComputeCableLength(0.0f, ToKinematicPitchAngle(GetPitchAngle()));
          pitch_zeroed = true;
          waiting_for_sz_after_pitch_limit = false;
          std::cout << "Turret limit switch pressed and SZ already zeroed: capturing turret angle reference"
                    << std::endl;
        } else {
          if (!waiting_for_sz_after_pitch_limit) {
            std::cout << "Turret limit switch pressed: holding pitch motor until SZ zero completes" << std::endl;
            waiting_for_sz_after_pitch_limit = true;
          }
        }
      } else {
        waiting_for_sz_after_pitch_limit = false;
        pitch_motor_->sendCommandMITMode(0.0, pitch_velocity, 0.0, 0.3, 0.0);
        if (!pitch_sign_flipped && loop_counter > 200 && std::fabs(current_pitch_vel) < 0.001) {
          pitch_sign_retries++;
          if (pitch_sign_retries >= 3) {
            pitch_velocity = -pitch_velocity;
            pitch_sign_flipped = true;
            std::cout << "ZeroTurret: pitch motor showed no motion, reversing pitch direction to "
                      << pitch_velocity << " rad/s" << std::endl;
          }
        }
      }
    }

    if (loop_counter % 100 == 0) {
      std::cout << "ZeroTurret progress: sz_count=" << spiral_zipper_.GetEncoderCount()
                << ", sz_zeroed=" << (sz_zeroed ? "YES" : "NO")
                << ", pitch_zeroed=" << (pitch_zeroed ? "YES" : "NO")
                << ", pitch_vel=" << current_pitch_vel
                << ", pitch_pos=" << current_pitch_pos
                << ", turret_limit_pressed=" << (turret_limit_pressed ? "YES" : "NO")
                << ", pitch_cmd=" << pitch_velocity << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  spiral_zipper_.Stop();
  pitch_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);

  return true;
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

double Turret::GetPitchAngle() const {
  return GetRawTurretEncoderAngle() - pitch_encoder_zero_angle_rad_ + pitch_angle_offset_rad_;
}

double Turret::GetPitchMotorAngle() const {
  if (pitch_motor_) {
    return static_cast<double>(pitch_motor_->getPosition());
  }
  return 0.0;
}

void Turret::EnterTeleopMode() {
  if (pitch_motor_) {
    pitch_motor_->enterMITMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "DEBUG: Pitch motor entered MIT mode for teleop" << std::endl;
  }
  if (yaw_motor_) {
    yaw_motor_->enterMITMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "DEBUG: Yaw motor entered MIT mode for teleop" << std::endl;
  }
}

void Turret::ExitTeleopMode() {
  StopAllMotors();
  if (pitch_motor_) {
    pitch_motor_->exitMITMode();
  }
  if (yaw_motor_) {
    yaw_motor_->exitMITMode();
  }
}

void Turret::SetSpiralZipperVelocity(double velocity) {
  spiral_zipper_.SetMotorVelocity(velocity);
}

void Turret::SetPitchVelocity(double velocity) {
  pitch_motor_->sendCommandMITMode(0.0, velocity, 0.0, 0.3, 0.0);
}

void Turret::SetYawVelocity(double velocity) {
  if (yaw_motor_) {
    yaw_motor_->sendCommandMITMode(0.0, velocity, 0.0, 0.3, 0.0);
  }
}

void Turret::StopAllMotors() {
  spiral_zipper_.Stop();
  if (pitch_motor_) {
    pitch_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);
  }
  if (yaw_motor_) {
    yaw_motor_->sendCommandMITMode(0.0, 0.0, 0.0, 0.5, 0.0);
  }
}

void Turret::ZeroPitchEncoder() {
  // Capture turret encoder reference at the current limit-switch angle.
  pitch_encoder_zero_angle_rad_ = GetRawTurretEncoderAngle();
  if (pitch_motor_) {
    pitch_motor_length_zero_angle_rad_ = pitch_motor_->getPosition();
    pitch_motor_cable_zero_length_m_ = ComputeCableLength(
        0.0f, ToKinematicPitchAngle(GetPitchAngle()));
  }
  std::cout << "DEBUG: Pitch encoder reference captured at "
            << pitch_encoder_zero_angle_rad_ * 180.0 / M_PI << " deg" << std::endl;
}

void Turret::ZeroYawMotor() {
  if (yaw_motor_) {
    yaw_motor_->zeroMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "DEBUG: Yaw motor zeroed" << std::endl;
  }
}

bool Turret::IsSpiralZipperLimitPressed() const {
  return spiral_zipper_.IsLimitSwitchPressed();
}

bool Turret::IsTurretLimitPressed() const {
  return turret_limit_switch_.IsPressed();
}

double Turret::GetYawAngle() const {
  if (yaw_motor_) {
    return static_cast<double>(yaw_motor_->getPosition());
  }
  return 0.0;
}

double Turret::GetYawMotorVelocity() const {
  if (yaw_motor_) {
    return static_cast<double>(yaw_motor_->getVelocity());
  }
  return 0.0;
}

double Turret::GetPitchMotorVelocity() const {
  if (pitch_motor_) {
    return static_cast<double>(pitch_motor_->getVelocity());
  }
  return 0.0;
}
