#ifndef TURRET_CONTROL_DOCKING_YAW_CONTROL_H_
#define TURRET_CONTROL_DOCKING_YAW_CONTROL_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace turret_control {

// Convert a camera-guided yaw request into one small, fixed position target.
// The hardware caller supplies position feedback and holds each returned yaw
// target. It must stop all motion when failed is set.
class DockingYawController {
 public:
  struct Config {
    double step_rad = 1.0 * kRadiansPerDegree;
    double target_deadband_rad = 0.2 * kRadiansPerDegree;
    double adjustment_timeout_sec = 2.0;
    double observation_pause_sec = 0.2;
    double yaw_limit_from_start_rad = 30.0 * kRadiansPerDegree;
    double camera_lateral_deadband_m = 0.005;
    double yaw_correction_sign = 1.0;
  };

  struct Input {
    double now_sec = 0.0;
    std::int64_t camera_now_ns = 0;
    std::int64_t pose_stamp_ns = 0;
    double yaw_rad = 0.0;
    double requested_yaw_rad = 0.0;
    double camera_x_error_m = 0.0;
    // A true tracking/phase interruption cancels the current position step.
    bool allow_alignment = false;
    // A post-burst capture wait prevents another step, but lets an existing
    // bounded position target finish without being replaced by measured yaw.
    bool allow_new_target = true;
  };

  struct Output {
    double yaw_rad = 0.0;
    double step_rad = 0.0;
    double step_motion_rad = 0.0;
    bool adjusting = false;
    bool waiting_for_capture = false;
    bool step_timed_out = false;
    bool failed = false;
    std::string status = "Yaw controller idle";
  };

  explicit DockingYawController(Config config) : config_(config) {}

  void reset() {
    output_ = Output{};
    started_ = false;
    start_yaw_rad_ = last_measured_yaw_rad_ = 0.0;
    last_time_sec_ = adjustment_start_sec_ = observation_start_sec_ = 0.0;
    capture_after_stamp_ns_ = 0;
    adjustment_capture_stamp_ns_ = 0;
    adjustment_direction_ = 0.0;
    adjustment_start_yaw_rad_ = 0.0;
  }

  void start(double now_sec, double measured_yaw_rad) {
    reset();
    if (!validConfig() || !nonnegative(now_sec) || !finite(measured_yaw_rad)) {
      fail("Invalid yaw configuration or starting feedback", measured_yaw_rad);
      return;
    }
    started_ = true;
    start_yaw_rad_ = last_measured_yaw_rad_ = output_.yaw_rad = measured_yaw_rad;
    last_time_sec_ = now_sec;
    output_.status = "Holding yaw measured at docking start";
  }

  Output update(const Input& input) {
    output_.step_timed_out = false;
    if (output_.failed) return output_;
    if (!started_) {
      return fail("Yaw controller has not been started", input.yaw_rad);
    }
    if (!validInput(input) || input.now_sec < last_time_sec_) {
      return fail("Invalid yaw input or non-monotonic time", input.yaw_rad);
    }
    if (std::fabs(input.yaw_rad - last_measured_yaw_rad_) >
        5.0 * kRadiansPerDegree) {
      return fail("Yaw feedback jumped by more than 5 degrees", input.yaw_rad);
    }
    if (input.yaw_rad < start_yaw_rad_ - config_.yaw_limit_from_start_rad ||
        input.yaw_rad > start_yaw_rad_ + config_.yaw_limit_from_start_rad) {
      return fail("Yaw feedback moved outside docking relative yaw bounds", input.yaw_rad);
    }
    last_time_sec_ = input.now_sec;
    last_measured_yaw_rad_ = input.yaw_rad;
    if (output_.adjusting) {
      output_.step_motion_rad = input.yaw_rad - adjustment_start_yaw_rad_;
    }

    const bool camera_centered = std::fabs(input.camera_x_error_m) <=
        config_.camera_lateral_deadband_m;
    if (!input.allow_alignment) {
      if (output_.adjusting) {
        return finishAdjustment(input, "Yaw step canceled; holding measured yaw for observation");
      }
      // Other docking phases may release the motor entirely. Refresh the held
      // yaw before strong position control resumes; keep any existing capture
      // gate without restarting its observation pause on every stopped tick.
      output_.yaw_rad = input.yaw_rad;
      if (!output_.waiting_for_capture) {
        output_.status = "Yaw alignment paused; holding measured yaw";
      }
      return output_;
    }
    if (output_.adjusting) {
      if (camera_centered) {
        return finishAdjustment(input, "Camera lateral goal reached; holding measured yaw for observation");
      }
      if (input.pose_stamp_ns > adjustment_capture_stamp_ns_) {
        adjustment_capture_stamp_ns_ = input.pose_stamp_ns;
        if (direction(config_.yaw_correction_sign * input.camera_x_error_m) !=
            adjustment_direction_) {
          return finishAdjustment(input, "Camera correction reversed; holding measured yaw for observation");
        }
      }
      if (adjustment_direction_ * (input.yaw_rad - output_.yaw_rad) >= 0.0) {
        return finishAdjustment(input, "Yaw step crossed its target; holding measured yaw for observation");
      }
      if (std::fabs(input.yaw_rad - output_.yaw_rad) <= config_.target_deadband_rad) {
        return finishAdjustment(input, "Yaw step reached; holding measured yaw for observation");
      }
      if (input.now_sec - adjustment_start_sec_ >= config_.adjustment_timeout_sec) {
        return finishAdjustment(input, "Yaw step timed out; holding measured yaw for observation", true);
      }
      // Neither a newer camera request nor repeated updates may accumulate a
      // farther target or restart the deadline while this step is in progress.
      // Waiting for a post-burst camera capture gates the next target only.
      return output_;
    }

    if (std::fabs(output_.yaw_rad - input.yaw_rad) >
        config_.step_rad + config_.target_deadband_rad) {
      // Do not apply high stiffness to a hold target left behind by drift or a
      // released motor. Capture the current yaw and observe before nudging.
      return finishAdjustment(input, "Measured yaw moved away from its hold; observing before another step");
    }
    if (!input.allow_new_target) {
      // Preserve the settled hold and any post-correction observation gate.
      // The next correction still needs both its pause and a new capture.
      if (!output_.waiting_for_capture) {
        output_.status = "Waiting for camera capture; holding yaw position";
      }
      return output_;
    }
    if (output_.waiting_for_capture) {
      if (input.now_sec - observation_start_sec_ < config_.observation_pause_sec ||
          input.pose_stamp_ns <= capture_after_stamp_ns_) {
        return output_;
      }
      output_.waiting_for_capture = false;
      output_.status = "Yaw observation complete; holding measured yaw";
    }
    if (camera_centered) {
      output_.yaw_rad = input.yaw_rad;
      output_.status = "Camera lateral goal is centered; holding measured yaw";
      return output_;
    }

    const double requested = std::clamp(input.requested_yaw_rad,
        start_yaw_rad_ - config_.yaw_limit_from_start_rad,
        start_yaw_rad_ + config_.yaw_limit_from_start_rad);
    const double request_error = requested - input.yaw_rad;
    // The core ramps its request in small increments. Use that request only
    // for direction/permission, not the step amplitude: a request barely
    // above the completion deadband produces almost no measurable movement.
    if (std::fabs(request_error) <= 1e-9) {
      return output_;
    }
    if (direction(request_error) !=
        direction(config_.yaw_correction_sign * input.camera_x_error_m)) {
      // The generic docking setpoint may still contain the previous camera
      // correction. Wait for its direction to match this current capture.
      output_.yaw_rad = input.yaw_rad;
      output_.status = "Holding measured yaw until request matches the current camera direction";
      return output_;
    }
    const double target = std::clamp(input.yaw_rad +
        direction(request_error) * config_.step_rad,
        start_yaw_rad_ - config_.yaw_limit_from_start_rad,
        start_yaw_rad_ + config_.yaw_limit_from_start_rad);
    if (std::fabs(target - input.yaw_rad) <= config_.target_deadband_rad) {
      return output_;
    }
    output_.yaw_rad = target;
    output_.step_rad = target - input.yaw_rad;
    output_.step_motion_rad = 0.0;
    output_.adjusting = true;
    adjustment_start_sec_ = input.now_sec;
    adjustment_start_yaw_rad_ = input.yaw_rad;
    adjustment_capture_stamp_ns_ = input.pose_stamp_ns;
    adjustment_direction_ = direction(target - input.yaw_rad);
    output_.status = "Adjusting yaw to one fixed small position step";
    return output_;
  }

 private:
  static constexpr double kRadiansPerDegree = 0.017453292519943295;
  Config config_;
  Output output_;
  bool started_ = false;
  double start_yaw_rad_ = 0.0;
  double last_measured_yaw_rad_ = 0.0;
  double last_time_sec_ = 0.0;
  double adjustment_start_sec_ = 0.0;
  double observation_start_sec_ = 0.0;
  std::int64_t capture_after_stamp_ns_ = 0;
  std::int64_t adjustment_capture_stamp_ns_ = 0;
  double adjustment_direction_ = 0.0;
  double adjustment_start_yaw_rad_ = 0.0;

  static bool finite(double value) { return std::isfinite(value); }
  static bool positive(double value) { return finite(value) && value > 0.0; }
  static bool nonnegative(double value) { return finite(value) && value >= 0.0; }
  static double direction(double value) { return value < 0.0 ? -1.0 : 1.0; }

  bool validConfig() const {
    return positive(config_.step_rad) && positive(config_.target_deadband_rad) &&
        config_.target_deadband_rad < config_.step_rad &&
        positive(config_.adjustment_timeout_sec) &&
        positive(config_.observation_pause_sec) &&
        positive(config_.yaw_limit_from_start_rad) &&
        config_.step_rad <= config_.yaw_limit_from_start_rad &&
        config_.yaw_limit_from_start_rad <= 180.0 * kRadiansPerDegree &&
        nonnegative(config_.camera_lateral_deadband_m) &&
        (config_.yaw_correction_sign == -1.0 || config_.yaw_correction_sign == 1.0);
  }

  static bool validInput(const Input& input) {
    return nonnegative(input.now_sec) && finite(input.yaw_rad) &&
        finite(input.requested_yaw_rad) && finite(input.camera_x_error_m) &&
        input.camera_now_ns >= 0 && input.pose_stamp_ns >= 0;
  }

  Output finishAdjustment(const Input& input, const char* status, bool timed_out = false) {
    output_.yaw_rad = input.yaw_rad;
    output_.adjusting = false;
    output_.waiting_for_capture = true;
    output_.step_timed_out = timed_out;
    observation_start_sec_ = input.now_sec;
    capture_after_stamp_ns_ = input.camera_now_ns;
    output_.status = status;
    return output_;
  }

  Output fail(const char* status, double measured_yaw_rad) {
    if (finite(measured_yaw_rad)) output_.yaw_rad = measured_yaw_rad;
    output_.adjusting = false;
    output_.waiting_for_capture = false;
    output_.failed = true;
    output_.status = status;
    return output_;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_DOCKING_YAW_CONTROL_H_
