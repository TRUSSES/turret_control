#ifndef TURRET_H_
#define TURRET_H_

#include "cubemars_pi3hat.h"
#include "encoder.h"
#include "limit_switch.h"
#include "spiral_zipper.h"
#include <yaml-cpp/yaml.h>
#include <memory>

class Turret {
 public:
  // Overloaded constructor that reads configuration from YAML.
  explicit Turret(const YAML::Node &node);
  
  ~Turret();

  void Init();
  bool ActuateTurretCable(float goal_dist, float desired_pitch_rad, float max_zipper_velocity,
                         bool hold_pitch = false);
  void Update();
  
  // Spiral zipper control methods for ROS2 integration
  void ZeroSpiralZipper();
  void ZeroSpiralZipper(double retract_velocity);
  void ActuateSpiralZipperLength(float goal_dist);
  void ActuateSpiralZipperLength(float goal_dist, double max_velocity);
  void StopSpiralZipper();
  double GetSpiralZipperExtension() const;
  double GetSpiralZipperVelocity() const;
  int GetSpiralZipperEncoderCount() const;

  double GetPitchAngle() const;

  // Combined zeroing routine for automatic zero sequence.
  // Returns true when both the spiral zipper limit and turret limit are reached.
  bool ZeroTurret(double retract_velocity, double pitch_velocity_ratio = 1.0);

  // Direct velocity control methods for teleop mode
  void EnterTeleopMode();
  void ExitTeleopMode();
  void SetSpiralZipperVelocity(double velocity);
  void SetPitchVelocity(double velocity);
  void SetYawVelocity(double velocity);
  void StopAllMotors();

  // Zeroing methods for teleop zero mode
  void ZeroPitchEncoder();          // Capture turret encoder reference at limit switch
  void ZeroYawMotor();              // Send zero command to yaw motor
  bool IsSpiralZipperLimitPressed() const;   // Check if SZ limit switch is pressed
  bool IsTurretLimitPressed() const;         // Check if turret limit switch is pressed

  // Position feedback methods
  double GetYawAngle() const;       // Get yaw position from motor feedback (radians)
  double GetPitchMotorAngle() const;  // Get pitch motor position (radians)
  double GetYawMotorVelocity() const;    // Get yaw motor velocity (rad/s)
  double GetPitchMotorVelocity() const;  // Get pitch motor velocity (rad/s)

 private:
  // Pi3Hat for all Cubemars motors (pitch motor, yaw motor)
  std::unique_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;

  Encoder turret_encoder_;  // Bourns EMS encoder for pitch angle
  LimitSwitch turret_limit_switch_;
  SpiralZipper spiral_zipper_;

  // Cubemars motors via Pi3Hat (pitch motor controls pitch cable, yaw motor controls yaw)
  std::unique_ptr<CubemarsPi3Hat> pitch_motor_;
  std::unique_ptr<CubemarsPi3Hat> yaw_motor_;

  float x_offset_;
  float y_offset_;
  double pitch_encoder_scale_rad_per_count_;
  double pitch_encoder_sign_;
  double pitch_kinematics_sign_;
  double pitch_encoder_zero_angle_rad_;
  double pitch_angle_offset_rad_;
  double pitch_motor_length_zero_angle_rad_;
  double pitch_motor_cable_zero_length_m_;

  const double cable_kp_ = 1.2;
  const double pitch_kp_ = 2.0;
  const double pitch_motor_radius_m_ = 0.013;
  const double max_omega_rad_ = 2.0;
  const double max_zipper_rate_coupled_ = 2.0;
  const double pitch_coupling_gain_ = 0.35;
  const double jacobian_damping_ = 0.02;

  double GetPitchCableLength() const;
  double ComputeCableLength(float extension, double pitch_angle_rad) const;
  void ComputeCableLengthJacobian(float extension,
                                 double pitch_angle_rad,
                                 double &dc_dz,
                                 double &dc_dtheta) const;

  double GetRawTurretEncoderAngle() const;
  double ToKinematicPitchAngle(double pitch_angle_rad) const;
};

#endif  // TURRET_H_
