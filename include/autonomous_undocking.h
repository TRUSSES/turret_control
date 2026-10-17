#ifndef TURRET_CONTROL_AUTONOMOUS_UNDOCKING_H_
#define TURRET_CONTROL_AUTONOMOUS_UNDOCKING_H_

#include "autonomous_docking.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace turret_control {

// Undocking uses joint encoders and the port's state, without camera feedback.
// The hardware adapter applies the stopped joint hold before publishing a
// one-shot port command, and enforces the extension floor during retraction.
class AutonomousUndockingController {
 public:
  using Input = AutonomousDockingController::Input;
  using DockCommand = AutonomousDockingController::DockCommand;

  enum class Phase {
    IDLE, DOCK_DEACTIVATE, DOCK_ACTIVATE, DOCK_OPEN, DOCK_RECOVER,
    SLOW_RETRACT, FAST_RETRACT, STOW_RETRACT, FINAL_PITCH, COMPLETE, FAILED
  };
  enum class Actuation { STOP, HOLD_PITCH_YAW, RETRACT_HOLD_PITCH, HOLD_FINAL_POSE };

  struct Config {
    double target_extension_m = 0.10;
    double target_pitch_rad = 20.0 * kRadiansPerDegree;
    double intermediate_extension_m = 0.30;
    double cruise_pitch_rad = 60.0 * kRadiansPerDegree;
    double slow_clearance_m = 0.10;
    double slow_velocity_rad_s = 1.2;
    double fast_velocity_rad_s = 2.0;
    double extension_tolerance_m = 0.005;
    double pitch_tolerance_rad = 2.0 * kRadiansPerDegree;
    double pitch_min_rad = 0.0;
    double pitch_max_rad = 63.0 * kRadiansPerDegree;
    double dock_deactivate_settle_sec = 2.0;
    double dock_activate_settle_sec = 2.0;
    double dock_open_settle_sec = 1.0;
    double dock_prepare_timeout_sec = 5.0;
    // Minimum time between port commands, including power-cycle retries.
    // Joint holds and feedback updates continue while a command is queued.
    double dock_command_interval_sec = 2.0;
    int dock_retry_attempts = 4;
  };

  struct Output {
    Actuation actuation = Actuation::STOP;
    Phase phase = Phase::IDLE;
    double extension_m = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    // Negative speed retracts. extension_m is the stop floor when retracting.
    double zipper_motor_velocity_rad_s = 0.0;
    DockCommand dock_command = DockCommand::NONE;
    int dock_retry_attempt = 0;
    std::string status = "Idle";
  };

  explicit AutonomousUndockingController(Config config) : config_(config) {}

  void reset() {
    output_ = Output{};
    phase_ = Phase::IDLE;
    resume_phase_ = Phase::SLOW_RETRACT;
    last_time_sec_ = phase_start_sec_ = 0.0;
    initial_extension_m_ = initial_pitch_rad_ = initial_yaw_rad_ = 0.0;
    slow_extension_m_ = 0.0;
    motion_started_ = cruise_pitch_started_ = false;
    pending_dock_command_ = DockCommand::NONE;
    last_dock_command_sec_ = 0.0;
    have_issued_dock_command_ = false;
    waiting_for_post_settle_state_ = false;
    dock_settle_sequence_ = recovery_state_sequence_ = 0;
  }

  void start(const Input& input) {
    reset();
    if (!validConfig() || !validInput(input)) {
      fail("Invalid undocking configuration or joint input");
      return;
    }
    last_time_sec_ = input.now_sec;
    if (input.pitch_rad < config_.pitch_min_rad || input.pitch_rad > config_.pitch_max_rad) {
      fail("Starting pitch is outside undocking motion bounds");
      return;
    }
    initial_extension_m_ = input.extension_m;
    initial_pitch_rad_ = input.pitch_rad;
    initial_yaw_rad_ = input.yaw_rad;
    slow_extension_m_ = std::max(config_.target_extension_m,
        initial_extension_m_ - config_.slow_clearance_m);
    output_.pitch_rad = initial_pitch_rad_;
    output_.yaw_rad = initial_yaw_rad_;
    enter(Phase::DOCK_DEACTIVATE, input.now_sec,
          "Undocking: deactivating dock before activating and opening latch");
    hold(input);
    pending_dock_command_ = DockCommand::IDLE;
  }

  Output update(const Input& input) {
    // The adapter must not publish a command cached from a previous update.
    output_.dock_command = DockCommand::NONE;
    if (phase_ == Phase::IDLE || phase_ == Phase::FAILED) return output_;
    if (!validInput(input) || input.now_sec < last_time_sec_) {
      return fail("Invalid undocking joint input or non-monotonic time");
    }
    last_time_sec_ = input.now_sec;
    if (phase_ == Phase::COMPLETE) {
      holdFinal(input);
      return output_;
    }
    if (motion_started_ && !cruise_pitch_started_ &&
        input.extension_m <= slow_extension_m_ + 1e-9) {
      // Keep the clearance milestone even if this update also loses port
      // feedback. Recovery holds the fixed cruise target, without recapturing
      // a measured angle that may have drifted during retraction.
      beginCruisePitch();
    }

    if (pending_dock_command_ != DockCommand::NONE) {
      hold(input);
      emitPendingDockCommand(input);
      return output_;
    }

    if (phase_ == Phase::DOCK_RECOVER) {
      hold(input);
      if (input.now_sec - phase_start_sec_ >= config_.dock_prepare_timeout_sec) {
        return fail("Undocking dock state did not recover; extension remains stopped");
      }
      if (!input.dock_fresh || input.dock_state_sequence <= recovery_state_sequence_) {
        return output_;
      }
      if (motion_started_ && portReady(input)) {
        enter(resume_phase_, input.now_sec, "Undocking dock feedback restored; resuming retraction");
        return motion(input);
      }
      return retryPort(input);
    }

    if (phase_ == Phase::DOCK_DEACTIVATE || phase_ == Phase::DOCK_ACTIVATE ||
        phase_ == Phase::DOCK_OPEN) {
      hold(input);
      if (input.now_sec - phase_start_sec_ >= config_.dock_prepare_timeout_sec) {
        return recover(input, "Undocking port preparation timed out");
      }
      const double settle = phase_ == Phase::DOCK_DEACTIVATE
          ? config_.dock_deactivate_settle_sec : phase_ == Phase::DOCK_ACTIVATE
          ? config_.dock_activate_settle_sec : config_.dock_open_settle_sec;
      if (!hasPostSettleState(input, settle) || input.dock_closing || input.dock_complete) {
        return output_;
      }
      if (phase_ == Phase::DOCK_DEACTIVATE) {
        if (!input.dock_idle || input.dock_relay_active) return output_;
        waiting_for_post_settle_state_ = false;
        enter(Phase::DOCK_ACTIVATE, input.now_sec,
              retryStatus("Undocking: dock deactivated; activating servos"));
        scheduleDockCommand(input, DockCommand::ACTIVATE);
        return output_;
      }
      if (!input.dock_active || !input.dock_relay_active) return output_;
      if (phase_ == Phase::DOCK_ACTIVATE) {
        if (input.dock_latch_open) return output_;
        waiting_for_post_settle_state_ = false;
        enter(Phase::DOCK_OPEN, input.now_sec,
              retryStatus("Undocking: dock activated; opening latch"));
        scheduleDockCommand(input, DockCommand::OPEN_LATCH);
        return output_;
      }
      if (!input.dock_latch_open) return output_;
      if (!motion_started_) {
        motion_started_ = true;
        enter(Phase::SLOW_RETRACT, input.now_sec,
              "Undocking: latch open; retracting slowly while holding pitch");
      } else {
        enter(resume_phase_, input.now_sec, "Undocking: latch reopened; resuming retraction");
      }
      return motion(input);
    }

    if (phase_ == Phase::SLOW_RETRACT || phase_ == Phase::FAST_RETRACT ||
        phase_ == Phase::STOW_RETRACT || phase_ == Phase::FINAL_PITCH) {
      if (!portReady(input)) {
        return recover(input, "Undocking port feedback or open-latch readiness lost");
      }
      return motion(input);
    }
    return fail("Unexpected autonomous undocking phase");
  }

  Phase phase() const { return phase_; }

 private:
  static constexpr double kRadiansPerDegree = 0.017453292519943295;
  Config config_;
  Output output_;
  Phase phase_ = Phase::IDLE;
  Phase resume_phase_ = Phase::SLOW_RETRACT;
  double last_time_sec_ = 0.0;
  double phase_start_sec_ = 0.0;
  double initial_extension_m_ = 0.0;
  double initial_pitch_rad_ = 0.0;
  double initial_yaw_rad_ = 0.0;
  double slow_extension_m_ = 0.0;
  DockCommand pending_dock_command_ = DockCommand::NONE;
  double last_dock_command_sec_ = 0.0;
  bool have_issued_dock_command_ = false;
  bool motion_started_ = false;
  bool cruise_pitch_started_ = false;
  bool waiting_for_post_settle_state_ = false;
  std::uint64_t dock_settle_sequence_ = 0;
  std::uint64_t recovery_state_sequence_ = 0;

  static bool finite(double value) { return std::isfinite(value); }
  static bool nonnegative(double value) { return finite(value) && value >= 0.0; }
  static bool positive(double value) { return finite(value) && value > 0.0; }

  bool validConfig() const {
    const double nonnegative_values[] = {
      config_.target_extension_m, config_.intermediate_extension_m,
      config_.slow_clearance_m, config_.extension_tolerance_m,
      config_.dock_deactivate_settle_sec, config_.dock_activate_settle_sec,
      config_.dock_open_settle_sec, config_.dock_command_interval_sec
    };
    for (double value : nonnegative_values) if (!nonnegative(value)) return false;
    const double positive_values[] = {
      config_.slow_velocity_rad_s, config_.fast_velocity_rad_s,
      config_.pitch_tolerance_rad, config_.dock_prepare_timeout_sec
    };
    for (double value : positive_values) if (!positive(value)) return false;
    return finite(config_.target_pitch_rad) && finite(config_.cruise_pitch_rad) &&
        finite(config_.pitch_min_rad) &&
        finite(config_.pitch_max_rad) && config_.pitch_min_rad < config_.pitch_max_rad &&
        config_.pitch_max_rad <= 63.0 * kRadiansPerDegree &&
        config_.pitch_max_rad - config_.pitch_min_rad >= kRadiansPerDegree &&
        config_.target_pitch_rad >= config_.pitch_min_rad &&
        config_.target_pitch_rad <= config_.pitch_max_rad &&
        config_.cruise_pitch_rad >= config_.pitch_min_rad &&
        config_.cruise_pitch_rad <= config_.pitch_max_rad &&
        config_.pitch_tolerance_rad < 90.0 * kRadiansPerDegree &&
        config_.extension_tolerance_m <= config_.target_extension_m &&
        config_.intermediate_extension_m >
            config_.target_extension_m + config_.extension_tolerance_m &&
        config_.slow_velocity_rad_s <= config_.fast_velocity_rad_s &&
        config_.fast_velocity_rad_s <= 2.0 && config_.dock_retry_attempts >= 0 &&
        config_.dock_prepare_timeout_sec > config_.dock_deactivate_settle_sec &&
        config_.dock_prepare_timeout_sec > config_.dock_activate_settle_sec &&
        config_.dock_prepare_timeout_sec > config_.dock_open_settle_sec;
  }

  static bool validInput(const Input& input) {
    return finite(input.now_sec) && nonnegative(input.extension_m) &&
        finite(input.pitch_rad) && finite(input.yaw_rad);
  }

  static bool portReady(const Input& input) {
    // The button may remain held while the turret starts pulling out. Only
    // retained closing/docked state prevents retraction after opening the latch.
    return input.dock_fresh && input.dock_active && input.dock_relay_active &&
        input.dock_latch_open && !input.dock_closing && !input.dock_complete;
  }

  void enter(Phase phase, double now, const std::string& status) {
    phase_ = output_.phase = phase;
    phase_start_sec_ = now;
    output_.status = status;
  }

  void hold(const Input& input) {
    output_.extension_m = input.extension_m;
    limitHeldPitch(input);
    output_.actuation = Actuation::HOLD_PITCH_YAW;
    output_.zipper_motor_velocity_rad_s = 0.0;
  }

  void holdFinal(const Input& input) {
    output_.extension_m = input.extension_m;
    output_.pitch_rad = config_.target_pitch_rad;
    limitHeldPitch(input);
    output_.yaw_rad = initial_yaw_rad_;
    output_.actuation = Actuation::HOLD_FINAL_POSE;
    output_.zipper_motor_velocity_rad_s = 0.0;
  }

  void scheduleDockCommand(const Input& input, DockCommand command) {
    pending_dock_command_ = command;
    waiting_for_post_settle_state_ = false;
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
    // Delay neither consumes operation timeout nor allows old completion
    // flags to acknowledge a command before it has been sent and settled.
    phase_start_sec_ = input.now_sec;
    waiting_for_post_settle_state_ = false;
  }

  Output fail(const std::string& status) {
    enter(Phase::FAILED, last_time_sec_, status);
    pending_dock_command_ = DockCommand::NONE;
    output_.dock_command = DockCommand::NONE;
    output_.actuation = Actuation::STOP;
    output_.zipper_motor_velocity_rad_s = 0.0;
    return output_;
  }

  Output motion(const Input& input) {
    if (phase_ != Phase::FINAL_PITCH &&
        input.extension_m <= config_.target_extension_m + config_.extension_tolerance_m) {
      beginCruisePitch();
      enter(Phase::FINAL_PITCH, input.now_sec,
            "Undocking: target extension reached; zipper stopped, setting final pitch");
    } else if (phase_ == Phase::SLOW_RETRACT &&
               input.extension_m <= slow_extension_m_ + 1e-9) {
      beginCruisePitch();
      enter(Phase::FAST_RETRACT, input.now_sec,
            "Undocking: clearance retracted; setting cruise pitch while retracting faster");
    }
    if (phase_ == Phase::FAST_RETRACT &&
        input.extension_m <= config_.intermediate_extension_m + config_.extension_tolerance_m) {
      enter(Phase::STOW_RETRACT, input.now_sec,
            "Undocking: intermediate extension reached; setting stow pitch while retracting");
    }
    resume_phase_ = phase_;
    output_.yaw_rad = initial_yaw_rad_;
    if (phase_ == Phase::FINAL_PITCH) {
      holdFinal(input);
      if (input.pitch_rad <= config_.pitch_max_rad &&
          std::fabs(input.pitch_rad - config_.target_pitch_rad) <= config_.pitch_tolerance_rad) {
        enter(Phase::COMPLETE, input.now_sec, "Undocking complete; holding final pitch and yaw");
      }
      return output_;
    }
    output_.actuation = Actuation::RETRACT_HOLD_PITCH;
    if (phase_ == Phase::SLOW_RETRACT) {
      output_.extension_m = slow_extension_m_;
      output_.pitch_rad = initial_pitch_rad_;
      output_.zipper_motor_velocity_rad_s = -config_.slow_velocity_rad_s;
    } else if (phase_ == Phase::FAST_RETRACT) {
      output_.extension_m = config_.intermediate_extension_m;
      output_.pitch_rad = config_.cruise_pitch_rad;
      output_.zipper_motor_velocity_rad_s = -config_.fast_velocity_rad_s;
    } else {
      output_.extension_m = config_.target_extension_m;
      output_.pitch_rad = config_.target_pitch_rad;
      output_.zipper_motor_velocity_rad_s = -config_.fast_velocity_rad_s;
    }
    if (cruise_pitch_started_ && input.pitch_rad > config_.pitch_max_rad) {
      // Preserve the stage and fixed reference. Retraction resumes only once
      // encoder feedback confirms pitch is back inside the configured ceiling.
      hold(input);
    }
    return output_;
  }

  void beginCruisePitch() {
    if (cruise_pitch_started_) return;
    cruise_pitch_started_ = true;
    output_.pitch_rad = config_.cruise_pitch_rad;
  }

  void limitHeldPitch(const Input& input) {
    if (!cruise_pitch_started_) return;
    double upper_target = config_.pitch_max_rad;
    if (input.pitch_rad > config_.pitch_max_rad) {
      // A target at the ceiling can lie inside the pitch motor's deadband even
      // when measured pitch is over it. Correct below the ceiling before moving.
      upper_target -= kRadiansPerDegree;
    }
    output_.pitch_rad = std::clamp(output_.pitch_rad,
                                  config_.pitch_min_rad, upper_target);
  }

  Output recover(const Input& input, const std::string& reason) {
    if (phase_ == Phase::SLOW_RETRACT || phase_ == Phase::FAST_RETRACT ||
        phase_ == Phase::STOW_RETRACT || phase_ == Phase::FINAL_PITCH) {
      resume_phase_ = phase_;
    }
    hold(input);
    recovery_state_sequence_ = input.dock_state_sequence;
    enter(Phase::DOCK_RECOVER, input.now_sec,
          reason + "; extension stopped while waiting for a new dock state");
    return output_;
  }

  std::string retryStatus(const std::string& status) const {
    return output_.dock_retry_attempt == 0 ? status :
        "Undocking retry " + std::to_string(output_.dock_retry_attempt) + "/" +
        std::to_string(config_.dock_retry_attempts) + ": " + status;
  }

  Output retryPort(const Input& input) {
    if (output_.dock_retry_attempt >= config_.dock_retry_attempts) {
      return fail("Undocking exhausted " + std::to_string(config_.dock_retry_attempts) +
                  " deactivate/activate retries");
    }
    ++output_.dock_retry_attempt;
    waiting_for_post_settle_state_ = false;
    enter(Phase::DOCK_DEACTIVATE, input.now_sec,
          retryStatus("Deactivating dock before activating and opening latch again"));
    scheduleDockCommand(input, DockCommand::IDLE);
    return output_;
  }

  bool hasPostSettleState(const Input& input, double settle_sec) {
    if (input.now_sec - phase_start_sec_ < settle_sec) return false;
    if (!waiting_for_post_settle_state_) {
      // There is no command ID in DockingState. Require a new receipt after the
      // physical settling wait, together with that stage's completion flags.
      dock_settle_sequence_ = input.dock_state_sequence;
      waiting_for_post_settle_state_ = true;
      return false;
    }
    return input.dock_fresh && input.dock_state_sequence > dock_settle_sequence_;
  }
};

}  // namespace turret_control

#endif  // TURRET_CONTROL_AUTONOMOUS_UNDOCKING_H_
