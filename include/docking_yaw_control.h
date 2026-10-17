#ifndef TURRET_CONTROL_DOCKING_YAW_CONTROL_H_
#define TURRET_CONTROL_DOCKING_YAW_CONTROL_H_

#include "docking_approach_profile.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace turret_control {

// Convert a camera-guided yaw request into one small, fixed position target.
// The hardware caller applies the returned motor velocity in a short,
// position-monitored pulse, then brakes and holds measured yaw before returning.
// It must stop all motion when failed is set. Feedback recovery stops extension
// and yaw driving while pitch can remain held.
// Recovery holds no motor against a single unconfirmed position outlier.
class DockingYawController {
 public:
  struct Config {
    DockingApproachProfile approach_profile;
    double step_rad = 3.0 * kRadiansPerDegree;
    double near_step_rad = 1.5 * kRadiansPerDegree;
    double motor_velocity_rad_s = 3.5;
    double velocity_pulse_sec = 0.015;
    double near_velocity_pulse_sec = 0.008;
    double target_deadband_rad = 0.2 * kRadiansPerDegree;
    double adjustment_timeout_sec = 2.0;
    double observation_pause_sec = 0.2;
    // Coarse alignment farther away avoids repeated small motor corrections.
    double camera_lateral_deadband_m = 0.030;
    // Once aligned, require larger drift before resuming camera corrections.
    double camera_lateral_resume_m = 0.050;
    // Tighten alignment on the near approach, with a separate restart window.
    double near_camera_lateral_deadband_m = 0.010;
    double near_camera_lateral_resume_m = 0.020;
    double yaw_correction_sign = 1.0;
  };

  struct Input {
    double now_sec = 0.0;
    std::int64_t camera_now_ns = 0;
    std::int64_t pose_stamp_ns = 0;
    double yaw_rad = 0.0;
    // Retained for callers sharing the generic docking input. Visual yaw
    // targets use current camera error, rather than this lagging setpoint.
    double requested_yaw_rad = 0.0;
    double camera_x_error_m = 0.0;
    // Zero means unavailable, retaining the original far-approach behavior.
    double camera_z_m = 0.0;
    // Increase only when a new CAN reply is received. Re-reading cached
    // feedback must not confirm a suspected discontinuity.
    std::uint64_t feedback_sequence = 0;
    // A large in-burst discontinuity needs confirmation even if the stopped
    // reply has returned near the prior trusted position. Small recoil and a
    // recovered brief CAN delay do not force this recovery path.
    bool force_feedback_confirmation = false;
    // Visual alignment must also observe a capture made after feedback
    // recovery. Search and insertion only require confirmed CAN feedback.
    bool require_recovery_capture = false;
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
    // Nonzero only for an active bounded correction. This is the motor-side
    // velocity used in teleop, not a continuous turret yaw-rate command.
    double motor_velocity_rad_s = 0.0;
    // Freeze pulse length with the active position target. A new depth estimate
    // must not change an in-progress correction or its timeout.
    double velocity_pulse_sec = 0.0;
    bool adjusting = false;
    bool waiting_for_capture = false;
    bool step_timed_out = false;
    bool feedback_recovery_required = false;
    bool feedback_jump_detected = false;
    double feedback_jump_rad = 0.0;
    bool failed = false;
    std::string status = "Yaw controller idle";
  };

  explicit DockingYawController(Config config) : config_(config) {}

  void reset() {
    output_ = Output{};
    started_ = false;
    last_measured_yaw_rad_ = 0.0;
    last_time_sec_ = adjustment_start_sec_ = observation_start_sec_ = 0.0;
    capture_after_stamp_ns_ = 0;
    adjustment_capture_stamp_ns_ = 0;
    adjustment_direction_ = 0.0;
    adjustment_start_yaw_rad_ = 0.0;
    last_feedback_sequence_ = 0;
    recovery_candidate_yaw_rad_ = 0.0;
    recovery_confirmations_ = 0;
    recovering_feedback_ = false;
    recovering_capture_ = false;
    camera_lateral_aligned_ = false;
    near_alignment_ = false;
  }

  void start(double now_sec, double measured_yaw_rad,
             std::uint64_t feedback_sequence = 0) {
    reset();
    if (!validConfig() || !nonnegative(now_sec) || !finite(measured_yaw_rad)) {
      fail("Invalid yaw configuration or starting feedback", measured_yaw_rad);
      return;
    }
    started_ = true;
    last_measured_yaw_rad_ = output_.yaw_rad = measured_yaw_rad;
    last_time_sec_ = now_sec;
    last_feedback_sequence_ = feedback_sequence;
    output_.status = "Holding yaw measured at docking start";
  }

  // Observe the latest reply, including replies generated while the hardware
  // caller executes a short yaw pulse. This keeps the jump baseline current
  // without starting or canceling an otherwise valid correction.
  Output observeFeedback(const Input& input) {
    output_.feedback_jump_detected = false;
    if (output_.failed) return output_;
    if (!started_) {
      return fail("Yaw controller has not been started", input.yaw_rad);
    }
    if (!validInput(input) || input.now_sec < last_time_sec_) {
      return fail("Invalid yaw input or non-monotonic time", input.yaw_rad);
    }
    last_time_sec_ = input.now_sec;
    const bool new_feedback = input.feedback_sequence > last_feedback_sequence_;
    if (new_feedback) {
      last_feedback_sequence_ = input.feedback_sequence;
      if (recovering_feedback_) {
        if (std::fabs(input.yaw_rad - recovery_candidate_yaw_rad_) >
            kRadiansPerDegree) {
          recovery_candidate_yaw_rad_ = input.yaw_rad;
          recovery_confirmations_ = 0;
          output_.status = "Yaw feedback is changing; stopped while confirming a stable position";
          return output_;
        }
        // Keep the candidate anchor fixed across confirming replies. A
        // slowly drifting sequence must not pass by moving this anchor.
        ++recovery_confirmations_;
        if (recovery_confirmations_ < 2) return output_;

        // Two distinct replies after the jump must agree before this measured
        // position becomes a hold target.
        last_measured_yaw_rad_ = input.yaw_rad;
        recovering_feedback_ = false;
        recovering_capture_ = input.require_recovery_capture;
        finishAdjustment(input,
            recovering_capture_
                ? "Yaw feedback confirmed; stopped while waiting for a new camera capture"
                : "Yaw feedback confirmed; holding the recovered measured position");
        output_.feedback_recovery_required = recovering_capture_;
        if (!recovering_capture_) output_.waiting_for_capture = false;
        return output_;
      }

      const double jump = input.yaw_rad - last_measured_yaw_rad_;
      if (std::fabs(jump) > 5.0 * kRadiansPerDegree || input.force_feedback_confirmation) {
        output_.yaw_rad = last_measured_yaw_rad_;
        output_.motor_velocity_rad_s = 0.0;
        output_.adjusting = false;
        output_.waiting_for_capture = false;
        output_.step_timed_out = false;
        output_.feedback_recovery_required = true;
        output_.feedback_jump_detected = true;
        output_.feedback_jump_rad = jump;
        recovering_feedback_ = true;
        recovering_capture_ = false;
        recovery_candidate_yaw_rad_ = input.yaw_rad;
        recovery_confirmations_ = 0;
        output_.status = input.force_feedback_confirmation
            ? "Large yaw change during a pulse; stopped while confirming new CAN replies"
            : "Yaw feedback jumped; stopped while confirming new CAN replies";
        return output_;
      }
      last_measured_yaw_rad_ = input.yaw_rad;
    }

    if (recovering_feedback_) return output_;
    if (recovering_capture_) {
      if (input.now_sec - observation_start_sec_ < config_.observation_pause_sec ||
          input.pose_stamp_ns <= capture_after_stamp_ns_) {
        return output_;
      }
      recovering_capture_ = false;
      output_.feedback_recovery_required = false;
      output_.waiting_for_capture = false;
      output_.yaw_rad = last_measured_yaw_rad_;
      output_.status = "Yaw feedback and camera observation recovered; holding measured yaw";
    }
    if (output_.adjusting) {
      output_.step_motion_rad = last_measured_yaw_rad_ - adjustment_start_yaw_rad_;
    }
    return output_;
  }

  Output update(const Input& received_input) {
    output_.motor_velocity_rad_s = 0.0;
    output_.step_timed_out = false;
    observeFeedback(received_input);
    if (output_.failed || output_.feedback_recovery_required) return output_;
    // Cached or duplicated CAN input is never allowed to replace the trusted
    // angle used for target completion and strong position holding.
    Input input = received_input;
    input.yaw_rad = last_measured_yaw_rad_;

    if (!input.allow_alignment) {
      if (output_.adjusting) {
        return finishAdjustment(input, "Yaw step canceled; holding measured yaw for observation");
      }
      // Preserve a settled hold instead of following each noisy encoder reply.
      // An interrupted active step captures its measured position above.
      if (!output_.waiting_for_capture) {
        output_.status = "Yaw alignment paused; holding settled yaw";
      }
      return output_;
    }
    // allow_alignment means a fresh visual pose in an approach phase. Once the
    // near goal is enabled, retain it even if depth jitters around its boundary.
    // Recheck a previous coarse alignment against the tighter near goal once.
    if (!near_alignment_ && input.pose_stamp_ns > 0 &&
        config_.approach_profile.nearRange(input.camera_z_m)) {
      near_alignment_ = true;
      camera_lateral_aligned_ = false;
    }
    const double lateral_deadband_m = near_alignment_
        ? config_.near_camera_lateral_deadband_m : config_.camera_lateral_deadband_m;
    const double lateral_resume_m = near_alignment_
        ? config_.near_camera_lateral_resume_m : config_.camera_lateral_resume_m;
    const double lateral_error_m = std::fabs(input.camera_x_error_m);
    if (lateral_error_m <= lateral_deadband_m) {
      camera_lateral_aligned_ = true;
    } else if (lateral_error_m > lateral_resume_m) {
      camera_lateral_aligned_ = false;
    }
    // Preserve the aligned state between the stop and resume thresholds. Camera
    // noise near the inner boundary must not queue more fine motor pulses.
    const bool camera_centered = camera_lateral_aligned_;
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
      output_.motor_velocity_rad_s = adjustment_direction_ * config_.motor_velocity_rad_s;
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
      output_.status = "Camera lateral goal is centered; holding settled yaw";
      return output_;
    }

    // The camera owns visual yaw direction. The core's independently ramped
    // setpoint may lag measured yaw after a step and must not veto the next
    // correction. Each target remains fixed until this step finishes.
    const double step_rad = config_.approach_profile.interpolate(
        config_.step_rad, config_.near_step_rad, input.camera_z_m);
    const double target = input.yaw_rad +
        direction(config_.yaw_correction_sign * input.camera_x_error_m) * step_rad;
    if (std::fabs(target - input.yaw_rad) <= config_.target_deadband_rad) {
      return output_;
    }
    output_.yaw_rad = target;
    output_.step_rad = target - input.yaw_rad;
    output_.step_motion_rad = 0.0;
    output_.velocity_pulse_sec = config_.approach_profile.interpolate(
        config_.velocity_pulse_sec, config_.near_velocity_pulse_sec, input.camera_z_m);
    output_.adjusting = true;
    adjustment_start_sec_ = input.now_sec;
    adjustment_start_yaw_rad_ = input.yaw_rad;
    adjustment_capture_stamp_ns_ = input.pose_stamp_ns;
    adjustment_direction_ = direction(target - input.yaw_rad);
    output_.motor_velocity_rad_s = adjustment_direction_ * config_.motor_velocity_rad_s;
    output_.status = "Adjusting yaw with a short velocity pulse toward one fixed position target";
    return output_;
  }

 private:
  static constexpr double kRadiansPerDegree = 0.017453292519943295;
  Config config_;
  Output output_;
  bool started_ = false;
  double last_measured_yaw_rad_ = 0.0;
  double last_time_sec_ = 0.0;
  double adjustment_start_sec_ = 0.0;
  double observation_start_sec_ = 0.0;
  std::int64_t capture_after_stamp_ns_ = 0;
  std::int64_t adjustment_capture_stamp_ns_ = 0;
  double adjustment_direction_ = 0.0;
  double adjustment_start_yaw_rad_ = 0.0;
  std::uint64_t last_feedback_sequence_ = 0;
  double recovery_candidate_yaw_rad_ = 0.0;
  int recovery_confirmations_ = 0;
  bool recovering_feedback_ = false;
  bool recovering_capture_ = false;
  bool camera_lateral_aligned_ = false;
  bool near_alignment_ = false;

  static bool finite(double value) { return std::isfinite(value); }
  static bool positive(double value) { return finite(value) && value > 0.0; }
  static bool nonnegative(double value) { return finite(value) && value >= 0.0; }
  static double direction(double value) { return value < 0.0 ? -1.0 : 1.0; }

  bool validConfig() const {
    return config_.approach_profile.valid() &&
        positive(config_.step_rad) && config_.step_rad <= 5.0 * kRadiansPerDegree &&
        positive(config_.near_step_rad) && config_.near_step_rad <= config_.step_rad &&
        positive(config_.motor_velocity_rad_s) && config_.motor_velocity_rad_s <= 5.0 &&
        positive(config_.velocity_pulse_sec) && config_.velocity_pulse_sec <= 0.05 &&
        positive(config_.near_velocity_pulse_sec) &&
        config_.near_velocity_pulse_sec <= config_.velocity_pulse_sec &&
        positive(config_.target_deadband_rad) && config_.target_deadband_rad <= kRadiansPerDegree &&
        config_.target_deadband_rad < config_.step_rad &&
        config_.target_deadband_rad < config_.near_step_rad &&
        positive(config_.adjustment_timeout_sec) &&
        positive(config_.observation_pause_sec) &&
        nonnegative(config_.camera_lateral_deadband_m) &&
        nonnegative(config_.camera_lateral_resume_m) &&
        config_.camera_lateral_resume_m >= config_.camera_lateral_deadband_m &&
        nonnegative(config_.near_camera_lateral_deadband_m) &&
        nonnegative(config_.near_camera_lateral_resume_m) &&
        config_.near_camera_lateral_deadband_m <= config_.camera_lateral_deadband_m &&
        config_.near_camera_lateral_resume_m <= config_.camera_lateral_resume_m &&
        config_.near_camera_lateral_resume_m >= config_.near_camera_lateral_deadband_m &&
        (config_.yaw_correction_sign == -1.0 || config_.yaw_correction_sign == 1.0);
  }

  static bool validInput(const Input& input) {
    return nonnegative(input.now_sec) && finite(input.yaw_rad) &&
        finite(input.requested_yaw_rad) && finite(input.camera_x_error_m) &&
        nonnegative(input.camera_z_m) &&
        input.camera_now_ns >= 0 && input.pose_stamp_ns >= 0;
  }

  Output finishAdjustment(const Input& input, const char* status, bool timed_out = false) {
    output_.yaw_rad = input.yaw_rad;
    output_.motor_velocity_rad_s = 0.0;
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
    output_.motor_velocity_rad_s = 0.0;
    output_.adjusting = false;
    output_.waiting_for_capture = false;
    output_.feedback_recovery_required = false;
    recovering_feedback_ = recovering_capture_ = false;
    output_.failed = true;
    output_.status = status;
    return output_;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_DOCKING_YAW_CONTROL_H_
