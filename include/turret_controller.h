#ifndef TURRET_CONTROLLER_H_
#define TURRET_CONTROLLER_H_

#include "turret.h"
#include <yaml-cpp/yaml.h>
#include <atomic>

class TurretController {
 public:
  // Constructor: accepts the full configuration YAML node.
  explicit TurretController(const YAML::Node &config);
  ~TurretController();

  // Runs the main control loop.
  void Spin();

  // Stops the control loop.
  void Stop();

  // Allows updating the desired goal parameters at runtime.
  void SetGoalParameters(float desired_extension, float desired_pitch, float desired_yaw);

 private:
  // The Turret object is built from the "turret" sub-node of the config.
  Turret turret_;

  // Running flag for the control loop.
  std::atomic<bool> running_;

  // Desired goal parameters for turret actuation.
  float desired_extension_;
  float desired_pitch_;
  float desired_yaw_;
};

#endif  // TURRET_CONTROLLER_H_
