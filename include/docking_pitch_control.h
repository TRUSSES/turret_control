#ifndef TURRET_CONTROL_DOCKING_PITCH_CONTROL_H_
#define TURRET_CONTROL_DOCKING_PITCH_CONTROL_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace turret_control {

// Schedule cable-motor speed from measured joint-pitch error. Large autonomous
// corrections move faster; small corrections retain the working movement floor.
// These speeds are motor rad/s, not turret joint angular rates.
class DockingPitchController {
 public:
  struct Config {
    double fine_velocity_rad_s = 0.25;
    double fine_error_rad = 0.03490658503988659;    // 2 deg
    double coarse_error_rad = 0.10471975511965977; // 6 deg
  };

  DockingPitchController() = default;

  explicit DockingPitchController(Config config) : config_(config) {
    if (!validConfig(config_)) {
      throw std::invalid_argument("Invalid autonomous pitch speed profile");
    }
  }

  static bool validConfig(const Config& config) {
    return std::isfinite(config.fine_velocity_rad_s) &&
        config.fine_velocity_rad_s >= kMinimumVelocityRadS &&
        config.fine_velocity_rad_s <= kMaximumVelocityRadS &&
        std::isfinite(config.fine_error_rad) &&
        config.fine_error_rad > kStopDeadbandRad &&
        std::isfinite(config.coarse_error_rad) &&
        config.coarse_error_rad > config.fine_error_rad;
  }

  double computeVelocity(double pitch_error_rad, double coarse_cap_rad_s,
                         bool lower_limit_pressed = false) const {
    if (!std::isfinite(pitch_error_rad) || !std::isfinite(coarse_cap_rad_s) ||
        coarse_cap_rad_s < kMinimumVelocityRadS ||
        coarse_cap_rad_s > kMaximumVelocityRadS) {
      throw std::invalid_argument(
          "Autonomous pitch requires a finite error and a speed cap from 0.20 to 2.0 rad/s");
    }

    const double error = std::fabs(pitch_error_rad);
    if (error <= kStopDeadbandRad ||
        (lower_limit_pressed && pitch_error_rad < 0.0)) {
      return 0.0;
    }

    // A caller may request a cap below the configured fine speed. Preserve its
    // cap and a monotonic profile by reducing the fine endpoint accordingly.
    const double fine_speed = std::min(config_.fine_velocity_rad_s, coarse_cap_rad_s);
    double speed = coarse_cap_rad_s;
    if (error <= config_.fine_error_rad) {
      speed = blend(kMinimumVelocityRadS, fine_speed,
          (error - kStopDeadbandRad) /
              (config_.fine_error_rad - kStopDeadbandRad));
    } else if (error < config_.coarse_error_rad) {
      speed = blend(fine_speed, coarse_cap_rad_s,
          (error - config_.fine_error_rad) /
              (config_.coarse_error_rad - config_.fine_error_rad));
    }
    return std::copysign(speed, pitch_error_rad);
  }

 private:
  static constexpr double kStopDeadbandRad = 0.008726646259971648; // 0.5 deg
  static constexpr double kMinimumVelocityRadS = 0.20;
  static constexpr double kMaximumVelocityRadS = 2.0;

  Config config_;

  static double blend(double low, double high, double fraction) {
    const double t = std::clamp(fraction, 0.0, 1.0);
    const double smooth = t * t * (3.0 - 2.0 * t);
    return low + (high - low) * smooth;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_DOCKING_PITCH_CONTROL_H_
