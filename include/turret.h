#ifndef TURRET_H_
#define TURRET_H_

#include "cubemars_control.h"
#include "encoder.h"
#include "limit_switch.h"
#include "spiral_zipper.h"
#include <yaml-cpp/yaml.h>
#include <chrono>

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

 private:
  Encoder turret_encoder_;
  LimitSwitch turret_limit_switch_;
  CubemarsControl pitch_motor_;
  CubemarsControl yaw_motor_;
  SpiralZipper spiral_zipper_;

  float x_offset_;
  float y_offset_;

  double prev_turret_angle_;
  std::chrono::time_point<std::chrono::steady_clock> last_turret_time_;

  const double cable_ff_ = 3.0;
  const double cable_kp_ = 1.2;
  const double spool_radius_ = 0.013;
  const double max_omega_rad_ = 6.0;
};

#endif  // TURRET_H_
