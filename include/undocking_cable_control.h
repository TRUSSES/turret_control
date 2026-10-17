#ifndef TURRET_CONTROL_UNDOCKING_CABLE_CONTROL_H_
#define TURRET_CONTROL_UNDOCKING_CABLE_CONTROL_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace turret_control {

// Match cable takeup to measured zipper travel, before joint-pitch error builds.
// Extension is measured in metres by the zipper encoder; no conversion from
// the separate zipper drive motor's angular velocity is assumed.
class UndockingCableController {
 public:
  void reset() {
    have_sample_ = false;
    sample_time_sec_ = sample_extension_m_ = extension_velocity_m_s_ = 0.0;
  }

  double computeVelocity(double now_sec, double extension_m,
                         double dc_dextension, double spool_radius_m,
                         double pitch_feedback_rad_s, double motor_cap_rad_s,
                         bool retracting, bool lower_limit_pressed = false) {
    if (!std::isfinite(now_sec) || !std::isfinite(extension_m) ||
        !std::isfinite(dc_dextension) || !std::isfinite(spool_radius_m) ||
        spool_radius_m <= 0.0 || !std::isfinite(pitch_feedback_rad_s) ||
        !std::isfinite(motor_cap_rad_s) || motor_cap_rad_s < 0.20 ||
        motor_cap_rad_s > 2.0) {
      reset();
      throw std::invalid_argument("Invalid undocking cable control input");
    }

    double feedforward_rad_s = 0.0;
    if (!retracting) {
      // A later restart must measure new travel, never reuse pre-stop motion.
      reset();
    } else {
      observeExtension(now_sec, extension_m);
      // db/dt = (db/dextension) * dextension/dt, motor speed = db/dt / r.
      feedforward_rad_s = dc_dextension * extension_velocity_m_s_ / spool_radius_m;
    }

    const double velocity = std::clamp(pitch_feedback_rad_s + feedforward_rad_s,
                                       -motor_cap_rad_s, motor_cap_rad_s);
    // Enforce the home-switch direction after summing both terms, including
    // feedforward when the ordinary pitch correction is in its deadband.
    return lower_limit_pressed && velocity < 0.0 ? 0.0 : velocity;
  }

 private:
  static constexpr double kMeasurementWindowSec = 0.020;
  static constexpr double kMaximumSampleGapSec = 0.100;

  bool have_sample_ = false;
  double sample_time_sec_ = 0.0;
  double sample_extension_m_ = 0.0;
  double extension_velocity_m_s_ = 0.0;

  void observeExtension(double now_sec, double extension_m) {
    const double dt = now_sec - sample_time_sec_;
    if (!have_sample_ || dt <= 0.0 || dt > kMaximumSampleGapSec) {
      // The first tick, clock resets, and long gaps establish a new baseline.
      have_sample_ = true;
      sample_time_sec_ = now_sec;
      sample_extension_m_ = extension_m;
      extension_velocity_m_s_ = 0.0;
      return;
    }
    if (dt < kMeasurementWindowSec) return;

    // Retraction must not pay cable out because of an isolated positive sample.
    // A stationary zipper produces zero, even if retraction is still requested.
    extension_velocity_m_s_ = std::min(0.0, (extension_m - sample_extension_m_) / dt);
    sample_time_sec_ = now_sec;
    sample_extension_m_ = extension_m;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_UNDOCKING_CABLE_CONTROL_H_
