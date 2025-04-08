#ifndef TURRET_CONTROLLER_H_
#define TURRET_CONTROLLER_H_

#include "turret.h"
#include <atomic>

class TurretController {
 public:
  // Constructs a TurretController.
  // The socket parameter is passed to the Turret's motor control.
  explicit TurretController(int socket);
  ~TurretController();

  // Spins the main control loop.
  // This method continuously updates the turret and commands actuation until stopped.
  void Spin();

  // Stops the control loop.
  void Stop();

  // Sets the desired goal parameters.
  //   desired_extension: desired spiral zipper extension (meters)
  //   desired_pitch: desired turret pitch (degrees)
  //   desired_yaw: desired turret yaw (degrees) [unused for now]
  void SetGoalParameters(float desired_extension, float desired_pitch, float desired_yaw);

 private:
  Turret turret_;
  std::atomic<bool> running_;

  // Goal parameters for turret actuation.
  float desired_extension_;
  float desired_pitch_;
  float desired_yaw_;
};

#endif  // TURRET_CONTROLLER_H_
