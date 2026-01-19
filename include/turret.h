#ifndef TURRET_H_
#define TURRET_H_

#include "cubemars_pi3hat.h"
#include "encoder.h"
#include "limit_switch.h"
#include "spiral_zipper.h"
#include "pi3hat.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>

// Velocity integration tracker for spool wire length
struct SpoolTracker {
    float wire_length_m = 0.0f;
    std::chrono::steady_clock::time_point last_update;
    bool initialized = false;

    // Calibration constants
    static constexpr float SPOOL_RADIUS_M = 0.013f;  // 26mm diameter
    static constexpr float VELOCITY_SCALE = 1.6f;    // Calibrated scale factor
    static constexpr float EXTENSION_SIGN = 1.0f;    // Direction sign

    void initialize() {
        wire_length_m = 0.0f;
        last_update = std::chrono::steady_clock::now();
        initialized = true;
    }

    void update(float measured_velocity_rad_s) {
        if (!initialized) return;

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_update).count();

        float corrected_velocity = measured_velocity_rad_s * EXTENSION_SIGN * VELOCITY_SCALE;
        float wire_rate_m_s = corrected_velocity * SPOOL_RADIUS_M;
        wire_length_m += wire_rate_m_s * dt;
        last_update = now;
    }

    float getWireLength() const { return wire_length_m; }
    void resetTimestamp() { last_update = std::chrono::steady_clock::now(); }
};

class Turret {
 public:
  // Constructor using explicit parameters.
  Turret(int socket, float x_offset = 0.0f, float y_offset = 0.0f);
  
  // Overloaded constructor that reads configuration from YAML.
  explicit Turret(const YAML::Node &node);
  
  ~Turret();

  void Init();
  bool ActuateTurretCable(float goal_dist, float desired_pitch_deg, float yaw);
  void Update();
  double GetTurretAngle() const;
  
  // Spiral zipper control methods for ROS2 integration
  void ZeroSpiralZipper();
  void ZeroSpiralZipper(double retract_velocity);
  void ActuateSpiralZipperLength(float goal_dist);
  void ActuateSpiralZipperLength(float goal_dist, double max_velocity);
  void StopSpiralZipper();
  double GetSpiralZipperExtension() const;
  double GetSpiralZipperVelocity() const;
  int GetSpiralZipperEncoderCount() const;

  // Spool control methods
  void ZeroSpool();
  void ActuateSpoolLength(float target_length_m, float rate_m_s = 0.05f);
  double GetSpoolWireLength() const;
  double GetPitchAngle() const;

  // Combined spiral zipper + spool control
  void ActuateCoupledExtension(float goal_dist, float desired_pitch_deg);

  // Direct velocity control methods for teleop mode
  void EnterTeleopMode();
  void ExitTeleopMode();
  void SetSpiralZipperVelocity(double velocity);
  void SetPitchVelocity(double velocity);
  void SetYawVelocity(double velocity);
  void StopAllMotors();

 private:
  // Pi3Hat for all Cubemars motors (pitch, yaw, spool)
  std::unique_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;

  Encoder turret_encoder_;
  Encoder pitch_encoder_;  // Bourns EMS encoder for pitch angle
  LimitSwitch turret_limit_switch_;
  SpiralZipper spiral_zipper_;

  // Cubemars motors via Pi3Hat
  std::unique_ptr<CubemarsPi3Hat> pitch_motor_;
  std::unique_ptr<CubemarsPi3Hat> yaw_motor_;
  std::unique_ptr<CubemarsPi3Hat> spool_motor_;
  SpoolTracker spool_tracker_;

  float x_offset_;
  float y_offset_;

  double prev_turret_angle_;
  std::chrono::time_point<std::chrono::steady_clock> last_turret_time_;

  const double cable_ff_ = 3.0;
  const double cable_kp_ = 1.2;
  const double spool_radius_ = 0.013;
  const double max_omega_rad_ = 6.0;

  // Pitch encoder scale (radians per count)
  static constexpr double kPitchAngleScale = 2.0 * 3.14159265359 / 1024.0;
};

#endif  // TURRET_H_
