#ifndef TURRET_H_
#define TURRET_H_

#include "cubemars_pi3hat.h"
#include "docking_pitch_control.h"
#include "encoder.h"
#include "limit_switch.h"
#include "spiral_zipper.h"
#include "undocking_cable_control.h"
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>
#include <string>

class Turret {
 public:
  // Overloaded constructor that reads configuration from YAML.
  explicit Turret(const YAML::Node &node);
  
  ~Turret();

  void Init();
  bool ActuateTurretCable(float goal_dist, float desired_pitch_rad, float max_zipper_velocity,
                         bool hold_pitch = false, bool prioritize_pitch = false,
                         double max_pitch_motor_velocity_rad_s = 0.0);
  bool ActuateFinalInsertionFreePitch(float goal_dist, float max_zipper_velocity);
  // Tune only autonomous docking/undocking pitch corrections and joint holds.
  void ConfigureDockingPitchControl(
      const turret_control::DockingPitchController::Config& config);
  // Center the measured turret pitch while keeping the zipper stopped.
  void ActuateDockingPitch(double target_pitch_rad,
                          double max_pitch_motor_velocity_rad_s);
  // Advance at motor velocity while holding pitch, bounded before insertion.
  void ActuateDockingAdvance(double zipper_velocity_rad_s, double hold_pitch_rad,
                            double max_pitch_motor_velocity_rad_s,
                            double max_extension_m);
  // Hold the turret pitch while inserting until the ROS docking controller stops it.
  // There is deliberately no insertion distance or elapsed-time cutoff.
  void ActuateDockingInsertion(double zipper_motor_velocity_rad_s, double hold_pitch_rad,
                              double max_pitch_motor_velocity_rad_s = 0.0);
  // Retract directly at negative motor velocity while controlling joint pitch.
  // Stop the zipper at the extension floor, home switch, or measured pitch cap;
  // keep correcting pitch while stopped. Measured zipper travel also drives
  // simultaneous cable takeup before a pitch error develops.
  void ActuateUndockingRetraction(double zipper_motor_velocity_rad_s,
                                 double target_pitch_rad, double minimum_extension_m,
                                 double max_pitch_motor_velocity_rad_s = 1.20,
                                 double maximum_pitch_rad = 1.0995574287564276); // 63 deg
  bool ActuateFixedVelocityPitchAndZipper(float goal_dist,
                                          float max_zipper_velocity,
                                          float pitch_motor_velocity_rad_s);
  void Update();
  // Refresh joint measurements without running the zipper motor controller.
  void UpdateEncoders();
  
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
  void ComputeEndEffectorPosition(double extension, double pitch_angle_rad,
                                  double &x, double &y) const;
  bool SolvePitchExtensionForPoint(double x, double y,
                                   double &extension, double &pitch_angle_rad) const;
  bool SolvePitchForHeightAtExtension(double y, double extension,
                                      double &pitch_angle_rad) const;

  // Combined zeroing routine for automatic zero sequence.
  // Returns true when both the spiral zipper limit and turret limit are reached.
  bool ZeroTurret(double retract_velocity, double pitch_velocity_ratio = 1.0);

  // Direct velocity control methods for teleop mode
  void EnterTeleopMode();
  void EnableControlMotors();
  void ExitTeleopMode();
  void SetSpiralZipperVelocity(double velocity);
  void SetPitchVelocity(double velocity);
  void SetYawVelocity(double velocity);
  void HoldYawPosition(double desired_angle_rad, double kp = 8.0, double kd = 2.0);
  struct DockingYawPulseResult {
    // A reply gap stopped the pulse; it does not invalidate a later fresh reply.
    bool feedback_interrupted = false;
    // True only when the stop command receives a new, fresh yaw reply. The
    // caller must still validate its position and drive status before holding.
    bool fresh_stopped_feedback = false;
    // A movement over five degrees needs independent position confirmation.
    bool position_discontinuity = false;
    // Largest travel opposite the requested direction, measured from the pulse
    // start. Small recoil is diagnostic and does not cancel the fixed target.
    double reverse_motion_rad = 0.0;
  };
  // Drive one monitored, short velocity burst toward a fixed yaw position.
  // The caller bounds target creation and keeps its original completion deadline.
  // Always brake before returning; cached replies cannot establish stopped
  // feedback. Small valid reverse motion continues toward the same target.
  // No velocity command remains active between calls.
  DockingYawPulseResult ActuateDockingYawVelocity(
      double signed_velocity_rad_s,
      double fixed_target_rad, double max_pulse_sec,
      double hold_kp, double hold_kd,
      double feedback_timeout_sec,
      double target_tolerance_rad = 0.003490658503988659); // 0.2 deg
  void StopAllMotors();
  void RequestZeroAbort();
  void ClearZeroAbort();

  // Zeroing methods for teleop zero mode
  void ZeroPitchEncoder();          // Capture turret encoder reference at limit switch
  void ZeroYawMotor();              // Send zero command to yaw motor
  bool IsSpiralZipperLimitPressed() const;   // Check if SZ limit switch is pressed
  bool IsTurretLimitPressed() const;         // Check if turret limit switch is pressed

  // Position feedback methods
  double GetYawAngle() const;       // Get yaw position from motor feedback (radians)
  bool HasFreshYawFeedback(double max_age_sec) const;
  std::uint64_t GetYawFeedbackSequence() const;  // Valid decoded yaw replies only
  std::string GetYawFeedbackDiagnostics() const;
  double GetPitchMotorAngle() const;  // Get pitch motor position (radians)
  double GetYawMotorVelocity() const;    // Get yaw motor velocity (rad/s)
  double GetYawMotorTorque() const;      // Get yaw motor torque feedback
  int GetYawMotorErrorFlag() const;      // Get yaw motor drive error flag
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
  const double pitch_kp_ = 2.4;
  const double pitch_motor_radius_m_ = 0.013;
  const double max_omega_rad_ = 2.0;
  const double max_zipper_rate_coupled_ = 2.0;
  const double pitch_coupling_gain_ = 0.35;
  const double jacobian_damping_ = 0.02;
  double final_insertion_pitch_motor_velocity_radps_ = 1.5;
  turret_control::DockingPitchController docking_pitch_controller_;
  turret_control::UndockingCableController undocking_cable_controller_;

  double GetPitchCableLength() const;
  double ComputeDockingPitchMotorVelocity(double target_pitch_rad,
                                          double current_pitch_rad,
                                          double max_pitch_motor_velocity_rad_s) const;
  void CapturePitchCableReferenceAtCurrentPose();
  void ResetCoordinatedTrajectory(double current_extension, double current_pitch_angle);
  void UpdateCoordinatedTrajectory(double goal_extension,
                                  double goal_pitch_angle,
                                  double max_zipper_velocity,
                                  bool hold_pitch,
                                  double current_extension,
                                  double current_pitch_angle,
                                  double &ref_extension,
                                  double &ref_pitch_angle,
                                  double &ref_extension_velocity,
                                  double &ref_pitch_velocity);
  double ComputeCableLength(float extension, double pitch_angle_rad) const;
  void ComputeCableLengthJacobian(float extension,
                                 double pitch_angle_rad,
                                 double &dc_dz,
                                 double &dc_dtheta) const;

  double GetRawTurretEncoderAngle() const;
  double ToKinematicPitchAngle(double pitch_angle_rad) const;

  bool coordinated_trajectory_active_ = false;
  bool coordinated_trajectory_initialized_ = false;
  bool coordinated_trajectory_hold_pitch_ = false;
  double coordinated_trajectory_start_extension_m_ = 0.0;
  double coordinated_trajectory_goal_extension_m_ = 0.0;
  double coordinated_trajectory_ref_extension_m_ = 0.0;
  double coordinated_trajectory_start_pitch_rad_ = 0.0;
  double coordinated_trajectory_goal_pitch_rad_ = 0.0;
  double coordinated_trajectory_ref_pitch_rad_ = 0.0;
  double coordinated_trajectory_ref_extension_velocity_mps_ = 0.0;
  double coordinated_trajectory_ref_pitch_velocity_radps_ = 0.0;
  double coordinated_trajectory_duration_s_ = 0.0;
  double coordinated_trajectory_progress_ = 1.0;
  double estimated_zipper_speed_mps_ = 0.04;
  double last_extension_sample_m_ = 0.0;
  bool have_last_extension_sample_ = false;
  std::atomic<bool> zero_abort_requested_{false};
  std::chrono::steady_clock::time_point coordinated_trajectory_last_update_;
  std::chrono::steady_clock::time_point last_extension_sample_time_;
};

#endif  // TURRET_H_
