#ifndef TURRET_CONTROL_DOCKING_APPROACH_PROFILE_H_
#define TURRET_CONTROL_DOCKING_APPROACH_PROFILE_H_

#include <algorithm>
#include <cmath>

namespace turret_control {

// Share camera-depth scaling between approach motion and yaw corrections.
struct DockingApproachProfile {
  double slowdown_start_m = 0.45;
  double slowdown_end_m = 0.25;

  bool valid() const {
    return std::isfinite(slowdown_start_m) && std::isfinite(slowdown_end_m) &&
        slowdown_end_m > 0.0 && slowdown_start_m > slowdown_end_m;
  }

  double nearBlend(double camera_z_m) const {
    // An unavailable camera depth retains the coarse profile for joint holds.
    if (!valid() || !std::isfinite(camera_z_m) || camera_z_m <= 0.0) return 0.0;
    const double fraction = std::clamp(
        (slowdown_start_m - camera_z_m) / (slowdown_start_m - slowdown_end_m),
        0.0, 1.0);
    return fraction * fraction * (3.0 - 2.0 * fraction);
  }

  double interpolate(double far_value, double near_value, double camera_z_m) const {
    return far_value + (near_value - far_value) * nearBlend(camera_z_m);
  }

  bool nearRange(double camera_z_m) const {
    return valid() && std::isfinite(camera_z_m) && camera_z_m > 0.0 &&
        camera_z_m <= slowdown_start_m;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_DOCKING_APPROACH_PROFILE_H_
