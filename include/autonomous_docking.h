#ifndef TURRET_CONTROL_AUTONOMOUS_DOCKING_H_
#define TURRET_CONTROL_AUTONOMOUS_DOCKING_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace turret_control {

// The caller supplies fresh camera coordinates and dock feedback. This
// controller owns setpoints; zeroing and hardware modes remain with the caller.
class AutonomousDockingController {
 public:
  enum class Phase {
    IDLE, SEARCH, PITCH_CENTER, APPROACH, WAIT_FOR_TAG,
    DOCK_RECOVER, DOCK_DEACTIVATE, DOCK_ACTIVATE, DOCK_OPEN, INSERT,
    LATCH_DEACTIVATE, LATCH_ACTIVATE, WAIT_FOR_LATCH, LATCH_RECOVER, COMPLETE, FAILED
  };
  enum class Actuation {
    STOP, CENTER_PITCH, TRACK_POSE, EXTEND_HOLD_PITCH, HOLD_PITCH_YAW, INSERT_HOLD_PITCH
  };
  enum class DockCommand { NONE, IDLE, ACTIVATE, OPEN_LATCH, CLOSE_LATCH };

  struct Config {
    double goal_camera_x_m = 0.0;
    double goal_camera_y_m = 0.0;
    double insertion_distance_m = 0.20;
    // Retained for parameter compatibility; lateral error no longer gates
    // the distance-triggered handoff. Yaw uses lateral_deadband_m on approach.
    double lateral_tolerance_m = 0.015;
    double lateral_deadband_m = 0.005;
    // Angular thresholds keep the tag near the image center at any distance.
    // Resume extension inside 1 degree; stop/re-center beyond 4 degrees.
    double centering_start_tolerance_rad = 1.0 * kRadiansPerDegree;
    double centering_stop_tolerance_rad = 4.0 * kRadiansPerDegree;
    double depth_deadband_m = 0.005;
    double pitch_min_rad = 0.0;
    double pitch_max_rad = 63.0 * kRadiansPerDegree;
    double yaw_limit_from_start_rad = 30.0 * kRadiansPerDegree;
    double pitch_down_sign = 1.0;
    // A positive sign increases motor yaw for positive camera x error.
    double yaw_correction_sign = 1.0;
    double pitch_gain_rad_per_m = 10.0;
    double yaw_gain = 1.5;
    double max_pitch_rate_rad_s = 5.0 * kRadiansPerDegree;
    double max_yaw_rate_rad_s = 5.0 * kRadiansPerDegree;
    double max_pitch_step_rad = 2.0 * kRadiansPerDegree;
    double max_yaw_step_rad = 1.0 * kRadiansPerDegree;
    double max_dt_sec = 0.1;
    double extension_min_m = 0.0;
    double extension_max_m = 0.90;
    double search_pitch_span_rad = 63.0 * kRadiansPerDegree;
    double search_pitch_rate_rad_s = 5.0 * kRadiansPerDegree;
    // Zipper motor angular speed, not Cartesian extension speed. The physical
    // zipper needs at least 1.2 rad/s to move. Approach uses short bursts at
    // this actual motor velocity, followed by a stopped camera/pitch check.
    double zipper_motor_velocity_rad_s = 1.2;
    double approach_burst_duration_sec = 2.0;
    // Power/open the port only at the camera handoff, with the zipper stopped.
    // These waits cover the port's servo initialization/opening operations.
    double dock_activate_settle_sec = 2.0;
    double dock_open_settle_sec = 1.0;
    double dock_prepare_timeout_sec = 5.0;
    double dock_deactivate_settle_sec = 2.0;
    // Space all port commands, including contact-triggered and retry cycles,
    // without blocking joint feedback or the immediate stop on contact.
    double dock_command_interval_sec = 2.0;
    // Additional cycles across preparation/insertion; latch closing has a
    // separate budget of the same size after its mandatory initial cycle.
    int dock_retry_attempts = 4;
    // Insertion holds the measured pitch and yaw at handoff. It intentionally
    // has no travel or time cutoff: fresh push-button contact ends insertion.
    double insertion_velocity_rad_s = 1.2;
    double latch_timeout_sec = 8.0;
    double latch_close_settle_sec = 1.0;
    int latch_closed_pulse_width_us = 1000;
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
    // Original camera capture stamp and current ROS clock share the same
    // synchronized clock domain. Motion/burst timing uses now_sec instead.
    std::int64_t pose_stamp_ns = 0;
    std::int64_t camera_now_ns = 0;
    bool dock_fresh = false;
    bool dock_detected = false;
    bool dock_closing = false;
    bool dock_complete = false;
    bool dock_active = false;
    bool dock_relay_active = false;
    bool dock_latch_open = false;
    bool dock_idle = false;
    bool dock_unoccupied = false;
    int dock_latch_pulse_width_us = 0;
    // Local receipt counter: require a state received after each settling wait.
    std::uint64_t dock_state_sequence = 0;
  };

  struct Output {
    Actuation actuation = Actuation::STOP;
    Phase phase = Phase::IDLE;
    double extension_m = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double zipper_motor_velocity_rad_s = 0.0;
    // A command is emitted only on the transition that needs it. The adapter
    // must publish it once, after stopping extension and applying joint holds.
    DockCommand dock_command = DockCommand::NONE;
    double handoff_camera_z_m = 0.0;
    int dock_retry_attempt = 0;
    int latch_retry_attempt = 0;
    std::string status = "Idle";
  };

  explicit AutonomousDockingController(Config config) : config_(config) {}

  void reset() {
    output_ = Output{};
    phase_ = Phase::IDLE;
    last_time_sec_ = phase_start_sec_ = 0.0;
    start_yaw_rad_ = hold_pitch_rad_ = hold_yaw_rad_ = 0.0;
    search_pitch_goal_rad_ = 0.0;
    center_after_stamp_ns_ = 0;
    waiting_for_center_capture_ = false;
    dock_settle_sequence_ = 0;
    waiting_for_dock_state_after_settle_ = false;
    recovery_state_sequence_ = 0;
    recovering_insertion_ = false;
    pending_dock_command_ = DockCommand::NONE;
    last_dock_command_sec_ = 0.0;
    have_issued_dock_command_ = false;
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
      fail("Starting pose is outside docking motion bounds: pitch=" +
           std::to_string(input.pitch_rad / kRadiansPerDegree) + " deg (allowed " +
           std::to_string(config_.pitch_min_rad / kRadiansPerDegree) + " to " +
           std::to_string(config_.pitch_max_rad / kRadiansPerDegree) + "), extension=" +
           std::to_string(input.extension_m) + " m (allowed " +
           std::to_string(config_.extension_min_m) + " to " +
           std::to_string(config_.extension_max_m) + ")");
      return;
    }
    // Existing contact/complete is not evidence for success of a new run,
    // including when the caller's last dock state is stale.
    if (input.dock_detected || input.dock_closing || input.dock_complete) {
      fail(std::string("Dock is already occupied or contact is already held: dock_detected=") +
           (input.dock_detected ? "true" : "false") + ", docking_state=" +
           (input.dock_complete ? "DOCKED" : input.dock_closing ? "DOCKING" : "UNDOCKED") +
           "; release the button and open the latch before starting a new attempt");
      return;
    }
    start_yaw_rad_ = hold_yaw_rad_ = input.yaw_rad;
    search_pitch_goal_rad_ = clampPitch(input.pitch_rad +
        config_.pitch_down_sign * config_.search_pitch_span_rad);
    enter(Phase::SEARCH, input.now_sec, "Searching for AprilTag with pitch down; extension held");
  }

  Output update(const Input& input) {
    output_.dock_command = DockCommand::NONE;
    if (phase_ == Phase::IDLE || phase_ == Phase::COMPLETE || phase_ == Phase::FAILED) {
      return output_;
    }
    if (!validJoints(input) || input.now_sec < last_time_sec_) {
      return fail("Invalid joint input or non-monotonic time");
    }
    const double dt = std::min(input.now_sec - last_time_sec_, config_.max_dt_sec);
    last_time_sec_ = input.now_sec;

    // Once contact is observed, stay in latch-only control even if IDLE clears
    // retained DOCKED or the button is released. Never resume insertion here.
    if (latchPhase()) {
      if (pending_dock_command_ != DockCommand::NONE) {
        holdForDock(input);
        emitPendingDockCommand(input);
        return output_;
      }
      return updateLatch(input);
    }
    // A reported DOCKED value is a timed command result, not a physical sensor.
    // Force one power cycle and explicit close rather than accepting that old
    // result when the servo may not actually have moved.
    if (input.dock_fresh && (input.dock_detected || input.dock_closing || input.dock_complete)) {
      freeze(input);
      return beginLatchCycle(input);
    }
    // Contact above may replace a queued OPEN/ACTIVATE with IDLE. Otherwise,
    // keep the turret held until the queued command can actually be sent;
    // no operation timeout or acknowledgement starts during this delay.
    if (pending_dock_command_ != DockCommand::NONE) {
      holdForDock(input);
      emitPendingDockCommand(input);
      return output_;
    }
    if (phase_ == Phase::DOCK_RECOVER) {
      holdForDock(input);
      if (input.now_sec - phase_start_sec_ >= config_.dock_prepare_timeout_sec) {
        return fail("Dock state did not recover; cannot confirm an unoccupied port for a power-cycle retry");
      }
      if (input.dock_fresh && input.dock_state_sequence > recovery_state_sequence_ &&
          input.dock_unoccupied) {
        if (recovering_insertion_ && dockReadyForInsertion(input)) {
          // A brief transport gap needs no latch reset. Resume only after a new
          // state confirms both the released button and the powered/open port.
          return insert(input);
        }
        return retryDock(input);
      }
      return output_;
    }
    if (phase_ == Phase::DOCK_DEACTIVATE || phase_ == Phase::DOCK_ACTIVATE ||
        phase_ == Phase::DOCK_OPEN) {
      // The captured pitch/yaw remains fixed through preparation and insertion.
      // A brief state gap during a stationary servo operation may wait here;
      // camera tracking is no longer required after the distance handoff.
      holdForDock(input);
      if (input.now_sec - phase_start_sec_ >= config_.dock_prepare_timeout_sec) {
        return recoverDock(input, false, phase_ == Phase::DOCK_DEACTIVATE
            ? "Timed out waiting for dock deactivation"
            : phase_ == Phase::DOCK_ACTIVATE ? "Timed out waiting for dock activation"
                                           : "Timed out waiting for dock latch to open");
      }
      const double settle_sec = phase_ == Phase::DOCK_DEACTIVATE
          ? config_.dock_deactivate_settle_sec : phase_ == Phase::DOCK_ACTIVATE
          ? config_.dock_activate_settle_sec : config_.dock_open_settle_sec;
      if (!hasPostSettleDockState(input, settle_sec)) return output_;
      if (!input.dock_unoccupied) return output_;
      if (phase_ == Phase::DOCK_DEACTIVATE) {
        if (!input.dock_idle || input.dock_relay_active) return output_;
        waiting_for_dock_state_after_settle_ = false;
        enter(Phase::DOCK_ACTIVATE, input.now_sec,
              retryStatus("Dock deactivated; activating again while holding pitch and yaw"));
        scheduleDockCommand(input, DockCommand::ACTIVATE);
        return output_;
      }
      if (!input.dock_active || !input.dock_relay_active) return output_;
      if (phase_ == Phase::DOCK_ACTIVATE) {
        // ACTIVATE initializes the latch closed. Do not send OPEN_LATCH until
        // activation has completed, including when old ACTIVE flags were set.
        if (input.dock_latch_open) return output_;
        waiting_for_dock_state_after_settle_ = false;
        enter(Phase::DOCK_OPEN, input.now_sec,
              retryStatus("Dock activated; opening latch while holding pitch and yaw"));
        scheduleDockCommand(input, DockCommand::OPEN_LATCH);
        return output_;
      }
      if (!input.dock_latch_open) return output_;
      return insert(input);
    }
    if (phase_ == Phase::INSERT) {
      if (!input.dock_fresh) {
        return recoverDock(input, true, "Dock feedback became stale during insertion");
      }
      if (!dockReadyForInsertion(input)) {
        return recoverDock(input, true, "Dock is no longer powered and open for insertion");
      }
      // Keep the pitch/yaw captured at handoff even when the tag is no longer
      // visible. Only extension moves until the push button reports contact.
      output_.extension_m = input.extension_m;
      output_.actuation = Actuation::INSERT_HOLD_PITCH;
      output_.zipper_motor_velocity_rad_s = config_.insertion_velocity_rad_s;
      output_.status = "Holding pitch and yaw; inserting slowly until push button contact";
      return output_;
    }

    if (input.pose_fresh && (!finite(input.camera_x_m) || !finite(input.camera_y_m) ||
        !finite(input.camera_z_m) || input.camera_z_m <= 0.0)) {
      freeze(input);
      return fail("Invalid fresh camera pose");
    }

    if (phase_ == Phase::SEARCH) {
      if (!input.pose_fresh) {
        // A bounded initial search may wait indefinitely at its pitch endpoint.
        // Never extend before a fresh tag has actually been acquired.
        return track(input, dt, search_pitch_goal_rad_, hold_yaw_rad_,
                     config_.search_pitch_rate_rad_s);
      }
      freeze(input);
      enter(Phase::PITCH_CENTER, input.now_sec, "Tag acquired; centering pitch before extension");
    } else if (phase_ == Phase::PITCH_CENTER || phase_ == Phase::APPROACH ||
               phase_ == Phase::WAIT_FOR_TAG) {
      if (!input.pose_fresh) {
        freeze(input);
        if (phase_ != Phase::WAIT_FOR_TAG) {
          enter(Phase::WAIT_FOR_TAG, input.now_sec, "Tracking unavailable; stopped while waiting for a fresh tag");
        }
        return stopped();
      }
      if (phase_ == Phase::WAIT_FOR_TAG) {
        // Discard the old lead over the encoders before resuming. A momentary
        // camera gap needs no retraction, sweep, retry counter or timeout.
        freeze(input);
        enter(Phase::PITCH_CENTER, input.now_sec, "Tag tracking restored; centering pitch before extension");
      }
    } else {
      return fail("Unexpected autonomous docking phase");
    }

    // A fresh close-range pose is the handoff. Do not wait for more lateral
    // alignment, pitch centering or a post-burst camera capture where the tag
    // is about to leave the usable camera range. Hold the measured joints while
    // powering and opening the port, then insert without further camera checks.
    if (atInsertionDistance(input.camera_z_m)) {
      if (!input.dock_fresh) {
        freeze(input);
        return fail("Fresh dock feedback is required before preparing insertion");
      }
      freeze(input);
      output_.handoff_camera_z_m = input.camera_z_m;
      waiting_for_dock_state_after_settle_ = false;
      enter(Phase::DOCK_ACTIVATE, input.now_sec,
            "Final insertion handoff; activating dock while holding pitch and yaw");
      holdForDock(input);
      scheduleDockCommand(input, DockCommand::ACTIVATE);
      return output_;
    }

    if (input.extension_m >= config_.extension_max_m) {
      freeze(input);
      return fail("Reached approach extension limit before insertion distance");
    }
    if (phase_ == Phase::APPROACH && burstExpired(input)) {
      // Always stop at the burst deadline, even if the last moving-camera
      // pose still looks centered. The next decision needs a capture made
      // after this stop, not another publication of the cached transform.
      freeze(input);
      hold_pitch_rad_ = input.pitch_rad;
      hold_yaw_rad_ = input.yaw_rad;
      center_after_stamp_ns_ = input.camera_now_ns;
      waiting_for_center_capture_ = true;
      enter(Phase::PITCH_CENTER, input.now_sec, "Extension burst complete; zipper stopped for a new camera capture and pitch check");
      return holdAfterBurst(input);
    }

    const double vertical_angle = std::fabs(verticalAngle(input));
    if (needsPitchCentering(input)) {
      // Stop the zipper directly: clearing its future position target alone
      // does not cancel the low-level extension trajectory immediately.
      freeze(input);
      enter(Phase::PITCH_CENTER, input.now_sec, "Tag left pitch centering window; zipper stopped for pitch adjustment");
    }
    if (phase_ == Phase::PITCH_CENTER) {
      if (waiting_for_center_capture_) {
        if (input.pose_stamp_ns <= center_after_stamp_ns_) {
          return holdAfterBurst(input);
        }
        waiting_for_center_capture_ = false;
      }
      if (vertical_angle > config_.centering_start_tolerance_rad) {
        return centerPitch(input, dt);
      }
      freeze(input);
      hold_pitch_rad_ = input.pitch_rad;
      enter(Phase::APPROACH, input.now_sec, "Tag centered; extending for a short burst while holding pitch");
    }

    return trackApproach(input, dt);
  }

  Phase phase() const { return phase_; }

  // A stopped burst must first receive a capture made after the stop. Holding
  // its old yaw setpoint during that wait is not a new alignment request.
  bool allowsYawAlignment(const Input& input) const {
    return input.pose_fresh &&
        (phase_ == Phase::PITCH_CENTER || phase_ == Phase::APPROACH) &&
        (!waiting_for_center_capture_ || input.pose_stamp_ns > center_after_stamp_ns_);
  }

  // Consume a valid near-range measurement in the fast hardware loop even
  // during pitch centering or a post-burst capture wait, before it expires.
  bool needsInsertionUpdate(const Input& input) const {
    return (phase_ == Phase::SEARCH || phase_ == Phase::PITCH_CENTER ||
            phase_ == Phase::APPROACH || phase_ == Phase::WAIT_FOR_TAG) &&
        input.pose_fresh && finite(input.camera_x_m) && finite(input.camera_y_m) &&
        positive(input.camera_z_m) && atInsertionDistance(input.camera_z_m);
  }

  // The hardware adapter may use this to stop extension as soon as a new
  // camera pose crosses the window, before its next setpoint-period update.
  bool needsPitchCentering(const Input& input) const {
    if (phase_ != Phase::APPROACH || !input.pose_fresh ||
        !finite(input.camera_y_m) || !positive(input.camera_z_m) ||
        atInsertionDistance(input.camera_z_m)) {
      return false;
    }
    return std::fabs(verticalAngle(input)) > config_.centering_stop_tolerance_rad;
  }

  // Stop a burst or react to approach bounds/handoff in the fast hardware loop,
  // without waiting for the usual pitch/yaw setpoint update period.
  bool needsApproachUpdate(const Input& input) const {
    return phase_ == Phase::APPROACH &&
        (burstExpired(input) || input.extension_m >= config_.extension_max_m ||
         (input.pose_fresh && positive(input.camera_z_m) &&
          atInsertionDistance(input.camera_z_m)));
  }

 private:
  static constexpr double kRadiansPerDegree = 0.017453292519943295;
  Config config_;
  Phase phase_ = Phase::IDLE;
  Output output_;
  double last_time_sec_ = 0.0;
  double phase_start_sec_ = 0.0;
  double start_yaw_rad_ = 0.0;
  double hold_pitch_rad_ = 0.0;
  double hold_yaw_rad_ = 0.0;
  double search_pitch_goal_rad_ = 0.0;
  std::int64_t center_after_stamp_ns_ = 0;
  bool waiting_for_center_capture_ = false;
  std::uint64_t dock_settle_sequence_ = 0;
  bool waiting_for_dock_state_after_settle_ = false;
  std::uint64_t recovery_state_sequence_ = 0;
  bool recovering_insertion_ = false;
  DockCommand pending_dock_command_ = DockCommand::NONE;
  double last_dock_command_sec_ = 0.0;
  bool have_issued_dock_command_ = false;

  static bool finite(double value) { return std::isfinite(value); }
  static bool positive(double value) { return finite(value) && value > 0.0; }
  static bool nonnegative(double value) { return finite(value) && value >= 0.0; }

  bool validConfig() const {
    const double finite_values[] = {
      config_.goal_camera_x_m, config_.goal_camera_y_m, config_.pitch_min_rad,
      config_.pitch_max_rad, config_.extension_min_m, config_.extension_max_m
    };
    for (double value : finite_values) if (!finite(value)) return false;
    const double positive_values[] = {
      config_.insertion_distance_m, config_.yaw_limit_from_start_rad,
      config_.max_pitch_rate_rad_s, config_.max_yaw_rate_rad_s,
      config_.max_pitch_step_rad, config_.max_yaw_step_rad, config_.max_dt_sec,
      config_.search_pitch_span_rad, config_.search_pitch_rate_rad_s,
      config_.zipper_motor_velocity_rad_s, config_.insertion_velocity_rad_s,
      config_.latch_timeout_sec, config_.centering_start_tolerance_rad,
      config_.centering_stop_tolerance_rad, config_.approach_burst_duration_sec,
      config_.dock_prepare_timeout_sec
    };
    for (double value : positive_values) if (!positive(value)) return false;
    const double nonnegative_values[] = {
      config_.lateral_tolerance_m, config_.lateral_deadband_m,
      config_.depth_deadband_m, config_.pitch_gain_rad_per_m, config_.yaw_gain,
      config_.dock_activate_settle_sec, config_.dock_open_settle_sec,
      config_.dock_deactivate_settle_sec, config_.latch_close_settle_sec,
      config_.dock_command_interval_sec
    };
    for (double value : nonnegative_values) if (!nonnegative(value)) return false;
    return (config_.pitch_down_sign == -1.0 || config_.pitch_down_sign == 1.0) &&
        (config_.yaw_correction_sign == -1.0 || config_.yaw_correction_sign == 1.0) &&
        config_.pitch_min_rad < config_.pitch_max_rad &&
        config_.extension_min_m >= 0.0 && config_.extension_min_m < config_.extension_max_m &&
        config_.lateral_deadband_m <= config_.lateral_tolerance_m &&
        config_.centering_start_tolerance_rad < config_.centering_stop_tolerance_rad &&
        config_.centering_stop_tolerance_rad < 90.0 * kRadiansPerDegree &&
        config_.dock_retry_attempts >= 0 &&
        config_.latch_closed_pulse_width_us > 0 &&
        config_.latch_timeout_sec > config_.latch_close_settle_sec &&
        config_.dock_prepare_timeout_sec > config_.dock_deactivate_settle_sec &&
        config_.dock_prepare_timeout_sec > config_.dock_activate_settle_sec &&
        config_.dock_prepare_timeout_sec > config_.dock_open_settle_sec;
  }

  static bool validJoints(const Input& input) {
    return finite(input.now_sec) && finite(input.extension_m) &&
        finite(input.pitch_rad) && finite(input.yaw_rad) &&
        input.pose_stamp_ns >= 0 && input.camera_now_ns >= 0;
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

  void enter(Phase phase, double now, const std::string& status) {
    phase_ = output_.phase = phase;
    phase_start_sec_ = now;
    output_.status = status;
  }

  Output stopped() {
    output_.actuation = Actuation::STOP;
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  Output fail(const std::string& status) {
    enter(Phase::FAILED, last_time_sec_, status);
    pending_dock_command_ = DockCommand::NONE;
    output_.dock_command = DockCommand::NONE;
    return stopped();
  }

  double verticalAngle(const Input& input) const {
    return std::atan2(config_.goal_camera_y_m - input.camera_y_m, input.camera_z_m);
  }

  bool atInsertionDistance(double camera_z_m) const {
    return camera_z_m <= config_.insertion_distance_m + config_.depth_deadband_m;
  }

  bool burstExpired(const Input& input) const {
    return input.now_sec - phase_start_sec_ >= config_.approach_burst_duration_sec;
  }

  void holdForDock(const Input& input) {
    output_.extension_m = input.extension_m;
    output_.actuation = Actuation::HOLD_PITCH_YAW;
    output_.zipper_motor_velocity_rad_s = 0.0;
  }

  void scheduleDockCommand(const Input& input, DockCommand command) {
    // A new command supersedes any pending one, especially when contact
    // interrupts preparation. The caller has already entered a stopped phase.
    pending_dock_command_ = command;
    waiting_for_dock_state_after_settle_ = false;
    emitPendingDockCommand(input);
  }

  void emitPendingDockCommand(const Input& input) {
    if (pending_dock_command_ == DockCommand::NONE ||
        (have_issued_dock_command_ && input.now_sec - last_dock_command_sec_ <
            config_.dock_command_interval_sec)) {
      return;
    }
    output_.dock_command = pending_dock_command_;
    pending_dock_command_ = DockCommand::NONE;
    last_dock_command_sec_ = input.now_sec;
    have_issued_dock_command_ = true;
    // Settling and acknowledgement are measured from actual publication,
    // rather than from the earlier phase transition that queued this command.
    phase_start_sec_ = input.now_sec;
    waiting_for_dock_state_after_settle_ = false;
  }

  static bool dockReadyForInsertion(const Input& input) {
    return input.dock_fresh && input.dock_unoccupied && input.dock_active &&
        input.dock_relay_active && input.dock_latch_open;
  }

  bool latchPhase() const {
    return phase_ == Phase::LATCH_DEACTIVATE || phase_ == Phase::LATCH_ACTIVATE ||
        phase_ == Phase::WAIT_FOR_LATCH || phase_ == Phase::LATCH_RECOVER;
  }

  std::string latchStatus(const std::string& status) const {
    return "Latch close attempt " + std::to_string(static_cast<long long>(output_.latch_retry_attempt) + 1) + "/" +
        std::to_string(static_cast<long long>(config_.dock_retry_attempts) + 1) + ": " + status;
  }

  Output beginLatchCycle(const Input& input) {
    holdForDock(input);
    waiting_for_dock_state_after_settle_ = false;
    enter(Phase::LATCH_DEACTIVATE, input.now_sec,
          latchStatus("Contact retained; deactivating dock before explicit latch close, zipper stopped"));
    scheduleDockCommand(input, DockCommand::IDLE);
    return output_;
  }

  Output updateLatch(const Input& input) {
    holdForDock(input);
    if (phase_ == Phase::LATCH_RECOVER) {
      if (input.now_sec - phase_start_sec_ >= config_.dock_prepare_timeout_sec) {
        return fail("Dock feedback did not return during latch recovery; turret stopped");
      }
      if (!input.dock_fresh || input.dock_state_sequence <= recovery_state_sequence_) return output_;
      if (output_.latch_retry_attempt >= config_.dock_retry_attempts) {
        return fail("Latch closing exhausted " + std::to_string(config_.dock_retry_attempts) +
                    " deactivate/activate retries");
      }
      ++output_.latch_retry_attempt;
      return beginLatchCycle(input);
    }
    const double timeout_sec = phase_ == Phase::WAIT_FOR_LATCH
        ? config_.latch_timeout_sec : config_.dock_prepare_timeout_sec;
    if (input.now_sec - phase_start_sec_ >= timeout_sec) {
      recovery_state_sequence_ = input.dock_state_sequence;
      enter(Phase::LATCH_RECOVER, input.now_sec,
            latchStatus("Latch command confirmation timed out; zipper stopped, waiting to retry"));
      return output_;
    }
    const double settle_sec = phase_ == Phase::LATCH_DEACTIVATE
        ? config_.dock_deactivate_settle_sec : phase_ == Phase::LATCH_ACTIVATE
        ? config_.dock_activate_settle_sec : config_.latch_close_settle_sec;
    if (!hasPostSettleDockState(input, settle_sec)) return output_;
    if (phase_ == Phase::LATCH_DEACTIVATE) {
      if (!input.dock_idle || input.dock_relay_active) return output_;
      waiting_for_dock_state_after_settle_ = false;
      enter(Phase::LATCH_ACTIVATE, input.now_sec,
            latchStatus("Dock deactivated; activating before closing latch"));
      scheduleDockCommand(input, DockCommand::ACTIVATE);
      return output_;
    }
    if (!input.dock_active || !input.dock_relay_active || input.dock_latch_open) return output_;
    if (phase_ == Phase::LATCH_ACTIVATE) {
      waiting_for_dock_state_after_settle_ = false;
      enter(Phase::WAIT_FOR_LATCH, input.now_sec,
            latchStatus("Dock active; sending CLOSE_LATCH with zipper stopped"));
      scheduleDockCommand(input, DockCommand::CLOSE_LATCH);
      return output_;
    }
    if (input.dock_latch_pulse_width_us != config_.latch_closed_pulse_width_us) return output_;
    enter(Phase::COMPLETE, input.now_sec,
          "Latch close command confirmed by dock feedback; turret stopped");
    return stopped();
  }

  Output insert(const Input& input) {
    output_.extension_m = input.extension_m;
    enter(Phase::INSERT, input.now_sec,
          "Holding pitch and yaw; inserting slowly until push button contact");
    output_.actuation = Actuation::INSERT_HOLD_PITCH;
    output_.zipper_motor_velocity_rad_s = config_.insertion_velocity_rad_s;
    return output_;
  }

  std::string retryStatus(const std::string& status) const {
    return output_.dock_retry_attempt == 0 ? status :
        "Dock retry " + std::to_string(output_.dock_retry_attempt) + "/" +
        std::to_string(config_.dock_retry_attempts) + ": " + status;
  }

  Output recoverDock(const Input& input, bool from_insertion, const std::string& reason) {
    holdForDock(input);
    recovery_state_sequence_ = input.dock_state_sequence;
    recovering_insertion_ = from_insertion;
    enter(Phase::DOCK_RECOVER, input.now_sec,
          reason + "; extension stopped, waiting for fresh dock state before resuming or retrying");
    return output_;
  }

  Output retryDock(const Input& input) {
    if (output_.dock_retry_attempt >= config_.dock_retry_attempts) {
      return fail("Dock recovery exhausted " + std::to_string(config_.dock_retry_attempts) +
                  " deactivate/activate retries");
    }
    ++output_.dock_retry_attempt;
    waiting_for_dock_state_after_settle_ = false;
    recovering_insertion_ = false;
    enter(Phase::DOCK_DEACTIVATE, input.now_sec,
          retryStatus("Deactivating dock before activating and opening again; holding pitch and yaw"));
    scheduleDockCommand(input, DockCommand::IDLE);
    return output_;
  }

  bool hasPostSettleDockState(const Input& input, double settle_sec) {
    if (input.now_sec - phase_start_sec_ < settle_sec) return false;
    if (!waiting_for_dock_state_after_settle_) {
      // Even an already ACTIVE/open port must provide a later state. The port
      // messages have no request ID, so combine this receipt gate with the
      // settling wait and the operation's expected completion flags.
      dock_settle_sequence_ = input.dock_state_sequence;
      waiting_for_dock_state_after_settle_ = true;
      return false;
    }
    return input.dock_fresh && input.dock_state_sequence > dock_settle_sequence_;
  }

  Output track(const Input& input, double dt, double pitch_goal,
               double yaw_goal, double pitch_rate) {
    output_.actuation = Actuation::TRACK_POSE;
    // Integrate targets so small 100 Hz increments cross actuator deadbands.
    // Limit the lead over measured joints so a blocked motor cannot accumulate
    // a large future movement when it becomes free.
    output_.extension_m = input.extension_m;
    output_.pitch_rad = std::clamp(boundedStep(output_.pitch_rad, clampPitch(pitch_goal),
        std::min(config_.max_pitch_step_rad, std::min(pitch_rate, config_.max_pitch_rate_rad_s) * dt)),
        input.pitch_rad - config_.max_pitch_step_rad,
        input.pitch_rad + config_.max_pitch_step_rad);
    output_.yaw_rad = std::clamp(boundedStep(output_.yaw_rad, clampYaw(yaw_goal),
        std::min(config_.max_yaw_step_rad, config_.max_yaw_rate_rad_s * dt)),
        input.yaw_rad - config_.max_yaw_step_rad,
        input.yaw_rad + config_.max_yaw_step_rad);
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  double yawRate(const Input& input) const {
    const double x_error = input.camera_x_m - config_.goal_camera_x_m;
    return std::fabs(x_error) > config_.lateral_deadband_m
        ? std::clamp(config_.yaw_correction_sign * config_.yaw_gain *
                     std::atan2(x_error, input.camera_z_m),
                     -config_.max_yaw_rate_rad_s, config_.max_yaw_rate_rad_s) : 0.0;
  }

  Output centerPitch(const Input& input, double dt) {
    const double pitch_rate = std::clamp(config_.pitch_gain_rad_per_m *
        (config_.goal_camera_y_m - input.camera_y_m),
        -config_.max_pitch_rate_rad_s, config_.max_pitch_rate_rad_s);
    output_.extension_m = input.extension_m;
    track(input, dt, output_.pitch_rad + pitch_rate * dt,
          output_.yaw_rad + yawRate(input) * dt, config_.max_pitch_rate_rad_s);
    output_.actuation = Actuation::CENTER_PITCH;
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  Output holdAfterBurst(const Input& input) {
    output_.extension_m = input.extension_m;
    output_.pitch_rad = hold_pitch_rad_;
    output_.yaw_rad = hold_yaw_rad_;
    output_.actuation = Actuation::CENTER_PITCH;
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  Output trackApproach(const Input& input, double dt) {
    // Hold the measured pitch captured when centering completed. Correct yaw
    // during extension; another pitch correction first stops the zipper.
    track(input, dt, hold_pitch_rad_,
          output_.yaw_rad + yawRate(input) * dt, config_.max_pitch_rate_rad_s);
    output_.actuation = Actuation::EXTEND_HOLD_PITCH;
    output_.zipper_motor_velocity_rad_s = config_.zipper_motor_velocity_rad_s;
    return output_;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_AUTONOMOUS_DOCKING_H_
