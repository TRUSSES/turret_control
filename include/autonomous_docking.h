#ifndef TURRET_CONTROL_AUTONOMOUS_DOCKING_H_
#define TURRET_CONTROL_AUTONOMOUS_DOCKING_H_

#include <algorithm>
#include <cmath>
#include <string>

namespace turret_control {

// The caller supplies fresh, filtered camera coordinates and dock feedback.
// This controller owns setpoints only; zeroing and hardware modes remain with
// the caller. STOP must stop all motors, including free-pitch insertion.
class AutonomousDockingController {
 public:
  enum class Phase {
    IDLE, SEARCH, ALIGN, APPROACH, RECOVERY_STOP, RECOVERY_RETRACT,
    RECOVERY_SWEEP, INSERT, WAIT_FOR_LATCH, COMPLETE, FAILED
  };
  enum class Actuation { STOP, TRACK_POSE, INSERT_FREE_PITCH };

  struct Config {
    double goal_camera_x_m = 0.0;
    double goal_camera_y_m = 0.03;
    double insertion_distance_m = 0.10;
    double lateral_tolerance_m = 0.015;
    double vertical_tolerance_m = 0.02;
    double lateral_deadband_m = 0.005;
    double vertical_deadband_m = 0.005;
    double depth_deadband_m = 0.005;
    double pitch_min_rad = 0.0;
    double pitch_max_rad = 63.0 * kRadiansPerDegree;
    double yaw_limit_from_start_rad = 30.0 * kRadiansPerDegree;
    double pitch_down_sign = 1.0;
    // A positive sign increases motor yaw for positive camera x error.
    double yaw_correction_sign = 1.0;
    double pitch_gain_rad_per_m = 10.0;
    double yaw_gain = 1.5;
    double extension_gain = 0.8;
    double max_pitch_rate_rad_s = 20.0 * kRadiansPerDegree;
    double max_yaw_rate_rad_s = 15.0 * kRadiansPerDegree;
    double max_extension_rate_m_s = 0.04;
    double max_pitch_step_rad = 2.0 * kRadiansPerDegree;
    double max_yaw_step_rad = 2.0 * kRadiansPerDegree;
    double max_extension_step_m = 0.01;
    double max_dt_sec = 0.1;
    double extension_min_m = 0.0;
    double extension_max_m = 0.90;
    // Negative means hold the extension measured at start. No initial extend
    // is required to begin looking for the tag.
    double initial_search_extension_m = -1.0;
    double search_pitch_span_rad = 25.0 * kRadiansPerDegree;
    double search_pitch_rate_rad_s = 8.0 * kRadiansPerDegree;
    double search_timeout_sec = 20.0;
    double alignment_timeout_sec = 20.0;
    double approach_timeout_sec = 40.0;
    // Zipper motor angular speed, not Cartesian extension speed.
    double zipper_motor_velocity_rad_s = 1.5;
    double recovery_retract_m = 0.03;
    double recovery_retract_velocity_rad_s = 0.5;
    double recovery_extension_tolerance_m = 0.003;
    double recovery_pitch_span_rad = 10.0 * kRadiansPerDegree;
    double recovery_pitch_rate_rad_s = 8.0 * kRadiansPerDegree;
    double recovery_pitch_endpoint_tolerance_rad = 1.0 * kRadiansPerDegree;
    double recovery_retract_timeout_sec = 8.0;
    double recovery_sweep_timeout_sec = 15.0;
    int max_recovery_attempts = 2;
    // Insertion intentionally has no travel or time cutoff: the push button
    // ends insertion. Loss of dock feedback still stops it immediately.
    double insertion_velocity_rad_s = 0.3;
    double latch_timeout_sec = 8.0;
  };

  struct Input {
    double now_sec = 0.0;
    double extension_m = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    bool pose_fresh = false;
    double camera_x_m = 0.0;
    double camera_y_m = 0.0;
    double camera_z_m = 0.0;
    bool dock_fresh = false;
    bool dock_detected = false;
    bool dock_closing = false;
    bool dock_complete = false;
  };

  struct Output {
    Actuation actuation = Actuation::STOP;
    Phase phase = Phase::IDLE;
    double extension_m = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double zipper_motor_velocity_rad_s = 0.0;
    std::string status = "Idle";
  };

  explicit AutonomousDockingController(Config config) : config_(config) {}

  void reset() {
    output_ = Output{};
    phase_ = Phase::IDLE;
    recovery_attempts_ = 0;
    have_pose_ = false;
    last_time_sec_ = phase_start_sec_ = 0.0;
    start_yaw_rad_ = hold_extension_m_ = hold_pitch_rad_ = hold_yaw_rad_ = 0.0;
    search_pitch_goal_rad_ = recovery_pitch_center_rad_ = 0.0;
    recovery_extension_goal_m_ = last_camera_z_m_ = 0.0;
    sweep_direction_ = 1.0;
  }

  void start(const Input& input) {
    reset();
    if (!validConfig() || !validJoints(input)) {
      fail("Invalid docking configuration or joint input");
      return;
    }
    freeze(input);
    last_time_sec_ = input.now_sec;
    if (input.extension_m < config_.extension_min_m ||
        input.extension_m > config_.extension_max_m ||
        input.pitch_rad < config_.pitch_min_rad ||
        input.pitch_rad > config_.pitch_max_rad) {
      fail("Starting pose is outside docking motion bounds");
      return;
    }
    // Refuse an occupied port even when the caller's last state is stale.
    // Existing contact/complete is never evidence for success of a new run.
    if (input.dock_detected || input.dock_closing || input.dock_complete) {
      fail("Dock is already occupied or contact is already held");
      return;
    }
    start_yaw_rad_ = input.yaw_rad;
    hold_extension_m_ = config_.initial_search_extension_m < 0.0
        ? input.extension_m : config_.initial_search_extension_m;
    hold_pitch_rad_ = input.pitch_rad;
    hold_yaw_rad_ = input.yaw_rad;
    search_pitch_goal_rad_ = clampPitch(input.pitch_rad +
        config_.pitch_down_sign * config_.search_pitch_span_rad);
    enter(Phase::SEARCH, input.now_sec, "Searching for AprilTag with pitch down");
  }

  Output update(const Input& input) {
    if (phase_ == Phase::IDLE || phase_ == Phase::COMPLETE || phase_ == Phase::FAILED) {
      return output_;
    }
    if (!validJoints(input) || input.now_sec < last_time_sec_) {
      return fail("Invalid joint input or non-monotonic time");
    }
    const double dt = std::min(input.now_sec - last_time_sec_, config_.max_dt_sec);
    last_time_sec_ = input.now_sec;

    // Contact wins over vision, phase deadlines, and movement setpoints.
    if (input.dock_fresh && input.dock_complete) {
      freeze(input);
      enter(Phase::COMPLETE, input.now_sec, "Dock latch confirmed complete");
      return stopped();
    }
    if (input.dock_fresh && (input.dock_detected || input.dock_closing) &&
        phase_ != Phase::WAIT_FOR_LATCH) {
      freeze(input);
      enter(Phase::WAIT_FOR_LATCH, input.now_sec, "Contact detected; motors stopped while latch closes");
      return stopped();
    }
    if (phase_ == Phase::INSERT || phase_ == Phase::WAIT_FOR_LATCH) {
      if (!input.dock_fresh) {
        freeze(input);
        return fail("Dock feedback became stale during insertion or latch wait");
      }
      if (phase_ == Phase::WAIT_FOR_LATCH) {
        if (expired(input, config_.latch_timeout_sec)) {
          return fail("Timed out waiting for dock latch completion");
        }
        return stopped();
      }
      output_.extension_m = input.extension_m;
      output_.actuation = Actuation::INSERT_FREE_PITCH;
      output_.zipper_motor_velocity_rad_s = config_.insertion_velocity_rad_s;
      output_.status = "Inserting slowly until push button contact";
      return output_;
    }

    if (input.pose_fresh && (!finite(input.camera_x_m) || !finite(input.camera_y_m) ||
        !finite(input.camera_z_m) || input.camera_z_m <= 0.0)) {
      freeze(input);
      return fail("Invalid fresh camera pose");
    }
    if (input.pose_fresh) {
      have_pose_ = true;
      last_camera_z_m_ = input.camera_z_m;
    }

    switch (phase_) {
      case Phase::SEARCH:
        if (input.pose_fresh) {
          hold_extension_m_ = input.extension_m;
          freeze(input);
          enter(Phase::ALIGN, input.now_sec, "Tag acquired; aligning yaw and pitch");
          return trackVision(input, dt, false);
        }
        if (expired(input, config_.search_timeout_sec)) {
          freeze(input);
          return fail("Pitch search timed out without a fresh tag");
        }
        return track(input, dt, hold_extension_m_, search_pitch_goal_rad_, hold_yaw_rad_,
                     config_.search_pitch_rate_rad_s, config_.zipper_motor_velocity_rad_s);

      case Phase::ALIGN:
      case Phase::APPROACH:
        if (!input.pose_fresh) {
          return beginRecovery(input);
        }
        if (phase_ == Phase::ALIGN && expired(input, config_.alignment_timeout_sec)) {
          freeze(input);
          return fail("Pitch and yaw alignment timed out");
        }
        if (phase_ == Phase::APPROACH && expired(input, config_.approach_timeout_sec)) {
          freeze(input);
          return fail("Vision-guided approach timed out");
        }
        if (aligned(input) && atInsertionDistance(input.camera_z_m)) {
          if (!input.dock_fresh) {
            freeze(input);
            return fail("Fresh dock feedback is required before insertion");
          }
          freeze(input);
          hold_yaw_rad_ = input.yaw_rad;
          enter(Phase::INSERT, input.now_sec, "Aligned at insertion distance; inserting until contact");
          output_.actuation = Actuation::INSERT_FREE_PITCH;
          output_.zipper_motor_velocity_rad_s = config_.insertion_velocity_rad_s;
          return output_;
        }
        if (phase_ == Phase::ALIGN && aligned(input)) {
          enter(Phase::APPROACH, input.now_sec, "Yaw and pitch aligned; approaching docking port");
        } else if (phase_ == Phase::APPROACH && !aligned(input)) {
          // Correct lateral/vertical errors before extending farther.
          hold_extension_m_ = input.extension_m;
          output_.extension_m = input.extension_m;
        }
        if (phase_ == Phase::APPROACH && aligned(input) &&
            input.extension_m >= config_.extension_max_m) {
          freeze(input);
          return fail("Reached approach extension limit before insertion distance");
        }
        return trackVision(input, dt, phase_ == Phase::APPROACH && aligned(input));

      case Phase::RECOVERY_STOP:
        enter(Phase::RECOVERY_RETRACT, input.now_sec, "Retracting before camera reacquisition");
        return track(input, dt, recovery_extension_goal_m_, hold_pitch_rad_, hold_yaw_rad_,
                     config_.max_pitch_rate_rad_s, config_.recovery_retract_velocity_rad_s);

      case Phase::RECOVERY_RETRACT:
        if (input.extension_m <= recovery_extension_goal_m_ +
            config_.recovery_extension_tolerance_m) {
          hold_extension_m_ = input.extension_m;
          freeze(input);
          enter(Phase::RECOVERY_SWEEP, input.now_sec, "Safe retraction complete; sweeping pitch for tag");
          // Publish the sweep phase once before accepting a reacquired tag.
          return trackSweep(input, dt);
        }
        if (expired(input, config_.recovery_retract_timeout_sec)) {
          freeze(input);
          return fail("Recovery retraction timed out");
        }
        return track(input, dt, recovery_extension_goal_m_, hold_pitch_rad_, hold_yaw_rad_,
                     config_.max_pitch_rate_rad_s, config_.recovery_retract_velocity_rad_s);

      case Phase::RECOVERY_SWEEP:
        if (input.pose_fresh) {
          hold_extension_m_ = input.extension_m;
          freeze(input);
          enter(Phase::ALIGN, input.now_sec, "Tag reacquired; realigning yaw and pitch");
          return trackVision(input, dt, false);
        }
        if (expired(input, config_.recovery_sweep_timeout_sec)) {
          freeze(input);
          return fail("Recovery pitch sweep timed out without a fresh tag");
        }
        return trackSweep(input, dt);

      default:
        return fail("Unexpected autonomous docking phase");
    }
  }

  Phase phase() const { return phase_; }

 private:
  static constexpr double kRadiansPerDegree = 0.017453292519943295;
  Config config_;
  Phase phase_ = Phase::IDLE;
  Output output_;
  double last_time_sec_ = 0.0;
  double phase_start_sec_ = 0.0;
  double start_yaw_rad_ = 0.0;
  double hold_extension_m_ = 0.0;
  double hold_pitch_rad_ = 0.0;
  double hold_yaw_rad_ = 0.0;
  double search_pitch_goal_rad_ = 0.0;
  double recovery_extension_goal_m_ = 0.0;
  double recovery_pitch_center_rad_ = 0.0;
  double sweep_direction_ = 1.0;
  int recovery_attempts_ = 0;
  bool have_pose_ = false;
  double last_camera_z_m_ = 0.0;

  static bool finite(double value) { return std::isfinite(value); }
  static bool positive(double value) { return finite(value) && value > 0.0; }
  static bool nonnegative(double value) { return finite(value) && value >= 0.0; }

  bool validConfig() const {
    const double finite_values[] = {
      config_.goal_camera_x_m, config_.goal_camera_y_m, config_.pitch_min_rad,
      config_.pitch_max_rad, config_.extension_min_m, config_.extension_max_m,
      config_.initial_search_extension_m
    };
    for (double value : finite_values) if (!finite(value)) return false;
    const double positive_values[] = {
      config_.insertion_distance_m, config_.yaw_limit_from_start_rad,
      config_.max_pitch_rate_rad_s, config_.max_yaw_rate_rad_s,
      config_.max_extension_rate_m_s, config_.max_pitch_step_rad,
      config_.max_yaw_step_rad, config_.max_extension_step_m, config_.max_dt_sec,
      config_.search_pitch_span_rad, config_.search_pitch_rate_rad_s,
      config_.search_timeout_sec, config_.alignment_timeout_sec,
      config_.approach_timeout_sec, config_.zipper_motor_velocity_rad_s,
      config_.recovery_retract_m, config_.recovery_retract_velocity_rad_s,
      config_.recovery_pitch_span_rad, config_.recovery_pitch_rate_rad_s,
      config_.recovery_pitch_endpoint_tolerance_rad,
      config_.recovery_retract_timeout_sec, config_.recovery_sweep_timeout_sec,
      config_.insertion_velocity_rad_s, config_.latch_timeout_sec
    };
    for (double value : positive_values) if (!positive(value)) return false;
    const double nonnegative_values[] = {
      config_.lateral_tolerance_m, config_.vertical_tolerance_m,
      config_.lateral_deadband_m, config_.vertical_deadband_m,
      config_.depth_deadband_m, config_.pitch_gain_rad_per_m, config_.yaw_gain,
      config_.extension_gain, config_.recovery_extension_tolerance_m
    };
    for (double value : nonnegative_values) if (!nonnegative(value)) return false;
    return (config_.pitch_down_sign == -1.0 || config_.pitch_down_sign == 1.0) &&
        (config_.yaw_correction_sign == -1.0 || config_.yaw_correction_sign == 1.0) &&
        config_.pitch_min_rad < config_.pitch_max_rad &&
        config_.extension_min_m >= 0.0 && config_.extension_min_m < config_.extension_max_m &&
        config_.lateral_deadband_m <= config_.lateral_tolerance_m &&
        config_.vertical_deadband_m <= config_.vertical_tolerance_m &&
        config_.recovery_extension_tolerance_m < config_.recovery_retract_m &&
        config_.recovery_pitch_endpoint_tolerance_rad < config_.recovery_pitch_span_rad &&
        config_.max_recovery_attempts >= 0 &&
        (config_.initial_search_extension_m < 0.0 ||
         (config_.initial_search_extension_m >= config_.extension_min_m &&
          config_.initial_search_extension_m <= config_.extension_max_m));
  }

  static bool validJoints(const Input& input) {
    return finite(input.now_sec) && finite(input.extension_m) &&
        finite(input.pitch_rad) && finite(input.yaw_rad);
  }

  double clampPitch(double pitch) const {
    return std::clamp(pitch, config_.pitch_min_rad, config_.pitch_max_rad);
  }

  double clampYaw(double yaw) const {
    return std::clamp(yaw, start_yaw_rad_ - config_.yaw_limit_from_start_rad,
                      start_yaw_rad_ + config_.yaw_limit_from_start_rad);
  }

  static double boundedStep(double current, double goal, double maximum_step) {
    return current + std::clamp(goal - current, -maximum_step, maximum_step);
  }

  void freeze(const Input& input) {
    output_.extension_m = input.extension_m;
    output_.pitch_rad = input.pitch_rad;
    output_.yaw_rad = input.yaw_rad;
  }

  void enter(Phase phase, double now, const char* status) {
    phase_ = output_.phase = phase;
    phase_start_sec_ = now;
    output_.status = status;
  }

  Output stopped() {
    output_.actuation = Actuation::STOP;
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  Output fail(const char* status) {
    enter(Phase::FAILED, last_time_sec_, status);
    return stopped();
  }

  bool expired(const Input& input, double timeout_sec) const {
    return input.now_sec - phase_start_sec_ >= timeout_sec;
  }

  bool aligned(const Input& input) const {
    return std::fabs(input.camera_x_m - config_.goal_camera_x_m) <= config_.lateral_tolerance_m &&
        std::fabs(input.camera_y_m - config_.goal_camera_y_m) <= config_.vertical_tolerance_m;
  }

  bool atInsertionDistance(double camera_z_m) const {
    return camera_z_m <= config_.insertion_distance_m + config_.depth_deadband_m;
  }

  Output track(const Input& input, double dt, double extension_goal, double pitch_goal,
               double yaw_goal, double pitch_rate, double zipper_velocity) {
    output_.actuation = Actuation::TRACK_POSE;
    // Integrate the previous target so 100 Hz increments can cross the inner
    // actuator's position deadband. Bound the target's lead over measured
    // joints, so a blocked motor cannot accumulate a large future movement.
    output_.extension_m = std::clamp(boundedStep(output_.extension_m,
        std::clamp(extension_goal, config_.extension_min_m, config_.extension_max_m),
        std::min(config_.max_extension_step_m, config_.max_extension_rate_m_s * dt)),
        input.extension_m - config_.max_extension_step_m,
        input.extension_m + config_.max_extension_step_m);
    output_.pitch_rad = std::clamp(boundedStep(output_.pitch_rad, clampPitch(pitch_goal),
        std::min(config_.max_pitch_step_rad, std::min(pitch_rate, config_.max_pitch_rate_rad_s) * dt)),
        input.pitch_rad - config_.max_pitch_step_rad,
        input.pitch_rad + config_.max_pitch_step_rad);
    output_.yaw_rad = std::clamp(boundedStep(output_.yaw_rad, clampYaw(yaw_goal),
        std::min(config_.max_yaw_step_rad, config_.max_yaw_rate_rad_s * dt)),
        input.yaw_rad - config_.max_yaw_step_rad,
        input.yaw_rad + config_.max_yaw_step_rad);
    output_.zipper_motor_velocity_rad_s = zipper_velocity;
    return output_;
  }

  Output trackVision(const Input& input, double dt, bool extend) {
    const double x_error = input.camera_x_m - config_.goal_camera_x_m;
    const double y_error = config_.goal_camera_y_m - input.camera_y_m;
    const double yaw_rate = std::fabs(x_error) > config_.lateral_deadband_m
        ? std::clamp(config_.yaw_correction_sign * config_.yaw_gain *
                     std::atan2(x_error, input.camera_z_m),
                     -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s) : 0.0;
    const double pitch_rate = std::fabs(y_error) > config_.vertical_deadband_m
        ? std::clamp(config_.pitch_gain_rad_per_m * y_error,
                     -config_.max_pitch_rate_rad_s, config_.max_pitch_rate_rad_s) : 0.0;
    double extension_goal = hold_extension_m_;
    if (extend) {
      const double depth_error = input.camera_z_m - config_.insertion_distance_m;
      const double extension_rate = depth_error > config_.depth_deadband_m
          ? std::clamp(config_.extension_gain * depth_error, 0.0,
                       config_.max_extension_rate_m_s) : 0.0;
      extension_goal = output_.extension_m + extension_rate * dt;
      hold_extension_m_ = input.extension_m;
    }
    return track(input, dt, extension_goal, output_.pitch_rad + pitch_rate * dt,
                 output_.yaw_rad + yaw_rate * dt, config_.max_pitch_rate_rad_s,
                 config_.zipper_motor_velocity_rad_s);
  }

  Output beginRecovery(const Input& input) {
    freeze(input);
    if (have_pose_ && atInsertionDistance(last_camera_z_m_)) {
      return fail("Stopped after close-range tracking loss");
    }
    if (recovery_attempts_ >= config_.max_recovery_attempts) {
      return fail("Camera recovery retry limit reached");
    }
    ++recovery_attempts_;
    hold_pitch_rad_ = input.pitch_rad;
    hold_yaw_rad_ = input.yaw_rad;
    recovery_pitch_center_rad_ = input.pitch_rad;
    recovery_extension_goal_m_ = std::max(config_.extension_min_m,
        input.extension_m - config_.recovery_retract_m);
    sweep_direction_ = config_.pitch_down_sign;
    enter(Phase::RECOVERY_STOP, input.now_sec, "Tracking lost; stopping before retraction");
    return stopped();
  }

  Output trackSweep(const Input& input, double dt) {
    const double low = clampPitch(recovery_pitch_center_rad_ - config_.recovery_pitch_span_rad);
    const double high = clampPitch(recovery_pitch_center_rad_ + config_.recovery_pitch_span_rad);
    if (input.pitch_rad >= high - config_.recovery_pitch_endpoint_tolerance_rad) sweep_direction_ = -1.0;
    if (input.pitch_rad <= low + config_.recovery_pitch_endpoint_tolerance_rad) sweep_direction_ = 1.0;
    return track(input, dt, hold_extension_m_, sweep_direction_ > 0.0 ? high : low,
                 hold_yaw_rad_, config_.recovery_pitch_rate_rad_s,
                 config_.recovery_retract_velocity_rad_s);
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_AUTONOMOUS_DOCKING_H_
