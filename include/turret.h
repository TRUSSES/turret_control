#ifndef TURRET_H_
#define TURRET_H_

#include <chrono>
#include "cubemars_control.h"
#include "encoder.h"
#include "limit_switch.h"
#include "spiral_zipper.h"

class Turret {
 public:
  // Constructs a Turret.
  // 'socket' is passed to the CubemarsControl instances.
  // x_offset and y_offset define geometric offsets (in meters).
  Turret(int socket, float x_offset = 0.0f, float y_offset = 0.0f);
  ~Turret();

  // Initializes turret components (e.g. zeroing the turret encoder).
  void Init();

  // Actuates the turret cable:
  //   goal_dist: desired spiral zipper extension (meters)
  //   desired_pitch_deg: desired turret pitch (in degrees)
  //   yaw: not used (to be implemented later)
  // Returns true if both spiral zipper and pitch motor have reached their targets.
  bool ActuateTurretCable(float goal_dist, float desired_pitch_deg, float yaw);

  // Updates turret sensors and the spiral zipper.
  void Update();

  // Returns the current turret pitch angle (in radians).
  double GetTurretAngle() const;

 private:
  // Turret pitch sensor and limit switch.
  Encoder turret_encoder_;       // Turret pitch encoder (e.g. cs=0, clk=6, do=5, max=1023)
  LimitSwitch turret_limit_switch_;  // Turret limit switch (e.g. pin 20)

  // Pitch and yaw motors (using CubemarsControl; yaw remains unimplemented).
  CubemarsControl pitch_motor_;
  CubemarsControl yaw_motor_;

  // Spiral zipper instance.
  SpiralZipper spiral_zipper_;

  // Geometric offsets.
  float x_offset_;
  float y_offset_;

  // For turret angular velocity estimation.
  double prev_turret_angle_;
  std::chrono::time_point<std::chrono::steady_clock> last_turret_time_;

  // Cable control gains and parameters.
  const double cable_ff_ = 3.0;    // Feed-forward gain
  const double cable_kp_ = 1.2;    // Proportional gain
  const double spool_radius_ = 0.013;  // Spool radius in meters
  const double max_omega_rad_ = 6.0;     // Maximum allowed angular velocity (rad/s)
};

#endif  // TURRET_H_
