#include "autonomous_undocking.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::AutonomousUndockingController;
using Phase = Controller::Phase;
using Actuation = Controller::Actuation;
using DockCommand = Controller::DockCommand;
constexpr double kDeg = 0.017453292519943295;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) check((expression), #expression, __LINE__)
bool near(double left, double right, double tolerance = 1e-9) {
  return std::fabs(left - right) <= tolerance;
}

Controller::Config fastFixtureConfig() {
  Controller::Config config;
  config.dock_command_interval_sec = 0.0;
  config.dock_deactivate_settle_sec = 0.3;
  config.dock_activate_settle_sec = 1.0;
  config.dock_open_settle_sec = 0.6;
  return config;
}

Controller::Config fixtureConfig(Controller::Config settings, bool use_command_delay) {
  // Historical state-machine scenarios keep their fast command fixture. The
  // spacing scenarios explicitly preserve the production/custom interval.
  if (!use_command_delay) settings.dock_command_interval_sec = 0.0;
  return settings;
}

struct Harness {
  Controller::Config config;
  Controller controller;
  Controller::Input input;
  double initial_pitch = 55.0 * kDeg;
  double initial_yaw = 0.2;

  explicit Harness(Controller::Config settings = fastFixtureConfig(), double extension = 0.60,
                   bool use_command_delay = false)
      : config(fixtureConfig(settings, use_command_delay)), controller(config) {
    input.extension_m = extension;
    input.pitch_rad = initial_pitch;
    input.yaw_rad = initial_yaw;
    input.dock_fresh = true;
    input.dock_active = input.dock_relay_active = true;
    input.dock_complete = input.dock_detected = true;
    input.dock_state_sequence = 1;
    // Undocking never needs a camera, even if every camera coordinate is invalid.
    input.camera_x_m = input.camera_y_m = input.camera_z_m =
        std::numeric_limits<double>::quiet_NaN();
    controller.start(input);
  }

  Controller::Output step(double dt = 0.01, bool new_state = false) {
    input.now_sec += dt;
    if (new_state) ++input.dock_state_sequence;
    return controller.update(input);
  }

  Controller::Output begin() {
    const auto output = step();
    CHECK(output.phase == Phase::DOCK_DEACTIVATE);
    CHECK(output.dock_command == DockCommand::IDLE);
    return output;
  }

  void idlePort() {
    input.dock_fresh = input.dock_idle = true;
    input.dock_active = input.dock_relay_active = input.dock_latch_open = false;
    input.dock_closing = input.dock_complete = false;
    // A button still physically held does not prevent opening for undocking.
    input.dock_detected = true;
    input.dock_unoccupied = false;
  }

  void readyPort() {
    input.dock_fresh = input.dock_active = input.dock_relay_active = input.dock_latch_open = true;
    input.dock_idle = input.dock_closing = input.dock_complete = false;
    input.dock_detected = true;
    input.dock_unoccupied = false;
  }

  Controller::Output deactivateToActivate() {
    CHECK(controller.phase() == Phase::DOCK_DEACTIVATE);
    idlePort();
    auto output = step(config.dock_deactivate_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::NONE);
    output = step(0.01, true);
    CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::ACTIVATE);
    return output;
  }

  Controller::Output activateToOpen() {
    CHECK(controller.phase() == Phase::DOCK_ACTIVATE);
    input.dock_idle = false;
    input.dock_active = input.dock_relay_active = true;
    input.dock_latch_open = false;
    auto output = step(config.dock_activate_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
    output = step(0.01, true);
    CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::OPEN_LATCH);
    return output;
  }

  Controller::Output openToMotion() {
    CHECK(controller.phase() == Phase::DOCK_OPEN);
    readyPort();
    auto output = step(config.dock_open_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::NONE);
    output = step(0.01, true);
    CHECK(output.dock_command == DockCommand::NONE);
    return output;
  }

  Controller::Output cycleToMotion() {
    deactivateToActivate();
    activateToOpen();
    return openToMotion();
  }

  Controller::Output motion() {
    begin();
    return cycleToMotion();
  }
};

void checkHold(const Controller::Output& output, double pitch, double yaw) {
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(near(output.pitch_rad, pitch));
  CHECK(near(output.yaw_rad, yaw));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
}

Controller::Output waitForSpacedCommand(Harness& harness, Phase phase,
                                      DockCommand command, double pitch,
                                      double& last_command_sec) {
  const double earliest = last_command_sec + harness.config.dock_command_interval_sec;
  for (int tick = 0; tick < 160; ++tick) {
    const auto output = harness.step(0.025);
    CHECK(output.phase == phase);
    checkHold(output, pitch, harness.initial_yaw);
    if (output.dock_command == DockCommand::NONE) {
      CHECK(harness.input.now_sec < earliest + 1e-9);
      continue;
    }
    CHECK(output.dock_command == command);
    CHECK(harness.input.now_sec + 1e-9 >= earliest);
    last_command_sec = harness.input.now_sec;
    CHECK(harness.step(0.0).dock_command == DockCommand::NONE);
    return output;
  }
  CHECK(false);
  return Controller::Output{};
}

Controller::Output acknowledgeAfterSettle(Harness& harness, double settle_sec) {
  auto output = harness.step(settle_sec + 0.001);
  CHECK(output.dock_command == DockCommand::NONE);
  return harness.step(0.01, true);
}

void testProductionCommandDwellBeforeUndocking() {
  Harness harness(Controller::Config{}, 0.60, true);
  CHECK(near(harness.config.dock_command_interval_sec, 2.0));
  CHECK(near(harness.config.dock_deactivate_settle_sec, 2.0));
  CHECK(near(harness.config.dock_activate_settle_sec, 2.0));
  CHECK(near(harness.config.dock_open_settle_sec, 1.0));
  CHECK(near(harness.config.slow_clearance_m, 0.10));
  CHECK(near(harness.config.fast_velocity_rad_s, 2.0));
  CHECK(near(harness.config.cruise_pitch_rad, 60.0 * kDeg));
  harness.begin();
  const double deactivated_sec = harness.input.now_sec;
  harness.deactivateToActivate();
  CHECK(harness.input.now_sec - deactivated_sec >= 2.0);
  const double activated_sec = harness.input.now_sec;
  harness.activateToOpen();
  CHECK(harness.input.now_sec - activated_sec >= 2.0);
  const auto output = harness.openToMotion();
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(near(output.zipper_motor_velocity_rad_s, -1.2));
}

void testUndockingCommandsAndRetryStaySpaced() {
  auto config = fastFixtureConfig();
  config.dock_command_interval_sec = 2.0;
  Harness harness(config, 0.60, true);
  harness.begin();
  double last_command_sec = harness.input.now_sec;
  harness.idlePort();
  auto output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  waitForSpacedCommand(harness, Phase::DOCK_ACTIVATE, DockCommand::ACTIVATE,
                      harness.initial_pitch, last_command_sec);
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
  CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::NONE);
  waitForSpacedCommand(harness, Phase::DOCK_OPEN, DockCommand::OPEN_LATCH,
                      harness.initial_pitch, last_command_sec);
  CHECK(harness.openToMotion().phase == Phase::SLOW_RETRACT);
  harness.input.extension_m = 0.51;
  harness.input.dock_latch_open = false;
  CHECK(harness.step(0.01).phase == Phase::DOCK_RECOVER);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 1);

  // A residual encoder movement may reach 10 cm while IDLE is delayed. The
  // fixed cruise pitch from that crossing must survive the entire queued retry.
  harness.input.extension_m = 0.50;
  harness.input.pitch_rad = 58.0 * kDeg;
  output = harness.step(0.01);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::NONE);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 59.0 * kDeg;
  output = waitForSpacedCommand(harness, Phase::DOCK_DEACTIVATE, DockCommand::IDLE,
                               60.0 * kDeg, last_command_sec);
  CHECK(output.dock_retry_attempt == 1);
  harness.idlePort();
  output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  waitForSpacedCommand(harness, Phase::DOCK_ACTIVATE, DockCommand::ACTIVATE,
                      60.0 * kDeg, last_command_sec);
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
  CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::NONE);
  waitForSpacedCommand(harness, Phase::DOCK_OPEN, DockCommand::OPEN_LATCH,
                      60.0 * kDeg, last_command_sec);
  output = harness.openToMotion();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  CHECK(output.dock_retry_attempt == 1);
}

void testQueuedUndockingStartsTimeoutAtEmission() {
  auto config = fastFixtureConfig();
  config.dock_command_interval_sec = 2.0;
  config.dock_activate_settle_sec = config.dock_open_settle_sec =
      config.dock_deactivate_settle_sec = 0.1;
  config.dock_prepare_timeout_sec = 0.6;
  Harness harness(config, 0.60, true);
  harness.begin();
  double last_command_sec = harness.input.now_sec;
  harness.idlePort();
  auto output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  harness.input.dock_fresh = false;
  // Waiting for the command interval consumes none of its operation timeout.
  output = harness.step(1.0);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  checkHold(output, harness.initial_pitch, harness.initial_yaw);
  waitForSpacedCommand(harness, Phase::DOCK_ACTIVATE, DockCommand::ACTIVATE,
                      harness.initial_pitch, last_command_sec);
  output = harness.step(0.59);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  CHECK(harness.step(0.02).phase == Phase::DOCK_RECOVER);
}

void testUndockingCommandIntervalValidation() {
  for (double invalid : {-0.01, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    auto config = fastFixtureConfig();
    config.dock_command_interval_sec = invalid;
    Harness harness(config, 0.60, true);
    const auto output = harness.step();
    CHECK(output.phase == Phase::FAILED && output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
  }
}

void testCommandOrderAndHeldButton() {
  Harness harness;
  auto output = harness.begin();
  checkHold(output, harness.initial_pitch, harness.initial_yaw);
  CHECK(harness.input.dock_complete); // Existing DOCKED is expected for undocking.
  output = harness.step();
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE);

  // The old ACTIVE/DOCKED response cannot acknowledge an IDLE command.
  output = harness.step(harness.config.dock_deactivate_settle_sec + 0.001, true);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::NONE);
  harness.idlePort();
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::ACTIVATE);
  checkHold(output, harness.initial_pitch, harness.initial_yaw);

  // Time alone, or an old receipt with completed flags, cannot advance the stage.
  harness.input.dock_active = harness.input.dock_relay_active = true;
  harness.input.dock_idle = false;
  output = harness.step(harness.config.dock_activate_settle_sec + 0.001);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(harness.step().phase == Phase::DOCK_ACTIVATE);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::OPEN_LATCH);
  output = harness.openToMotion();
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(output.actuation == Actuation::RETRACT_HOLD_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, -1.2));
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  CHECK(near(output.extension_m, 0.50));
  CHECK(harness.input.dock_detected && !harness.input.dock_unoccupied);
  CHECK(harness.step().dock_command == DockCommand::NONE);
}

void testMeasuredClearanceAndFinalPose() {
  Harness harness;
  harness.motion();
  // Elapsed time does not finish the initial 10 cm clearance retraction.
  auto output = harness.step(300.0);
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  harness.input.extension_m = 0.500001;
  harness.input.pitch_rad = 58.0 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  CHECK(near(output.zipper_motor_velocity_rad_s, -1.2));
  // Exactly 10 cm of measured retreat switches to the configured cruise pitch.
  harness.input.extension_m = 0.50;
  output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.yaw_rad, harness.initial_yaw));
  // Measurements may drift, but the cruise pitch must not become a moving target.
  harness.input.pitch_rad = 61.0 * kDeg;
  harness.input.extension_m = 0.40;
  output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  harness.input.extension_m = 0.306;
  CHECK(harness.step().phase == Phase::FAST_RETRACT);
  harness.input.extension_m = 0.304;
  output = harness.step();
  CHECK(output.phase == Phase::STOW_RETRACT);
  CHECK(output.actuation == Actuation::RETRACT_HOLD_PITCH);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  harness.input.extension_m = 0.106;
  CHECK(harness.step().phase == Phase::STOW_RETRACT);
  harness.input.extension_m = 0.104;
  output = harness.step();
  CHECK(output.phase == Phase::FINAL_PITCH);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  harness.input.pitch_rad = 23.0 * kDeg;
  CHECK(harness.step().phase == Phase::FINAL_PITCH);
  harness.input.pitch_rad = 21.9 * kDeg;
  harness.input.yaw_rad = 0.4;
  output = harness.step();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  CHECK(near(output.yaw_rad, harness.initial_yaw));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  harness.input.dock_fresh = false;
  output = harness.step();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(output.dock_command == DockCommand::NONE);
}

void testShortStartingExtensionNeverExtendsOrOverRetracts() {
  for (double extension : {0.30, 0.34, 0.35}) {
    Harness harness({}, extension);
    auto output = harness.motion();
    CHECK(output.phase == Phase::SLOW_RETRACT);
    harness.input.extension_m = extension - 0.099;
    output = harness.step();
    CHECK(output.phase == Phase::SLOW_RETRACT);
    CHECK(near(output.pitch_rad, harness.initial_pitch));
    harness.input.extension_m = extension - 0.10;
    output = harness.step();
    CHECK(output.phase == Phase::STOW_RETRACT);
    CHECK(near(output.pitch_rad, 20.0 * kDeg));
    CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
    CHECK(near(output.extension_m, 0.10));
    harness.input.extension_m = 0.104;
    output = harness.step();
    CHECK(output.phase == Phase::FINAL_PITCH);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  }
  // With less than 10 cm available, the entire retreat stays slow to the final floor.
  for (double extension : {0.15, 0.19}) {
    Harness harness({}, extension);
    auto output = harness.motion();
    CHECK(output.phase == Phase::SLOW_RETRACT);
    harness.input.extension_m = 0.106;
    output = harness.step();
    CHECK(output.phase == Phase::SLOW_RETRACT);
    CHECK(near(output.pitch_rad, harness.initial_pitch));
    harness.input.extension_m = 0.104;
    output = harness.step();
    CHECK(output.phase == Phase::FINAL_PITCH);
    CHECK(near(output.pitch_rad, 20.0 * kDeg));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  }
  for (double extension : {0.09, 0.10}) {
    Harness harness({}, extension);
    const auto output = harness.motion();
    CHECK(output.phase == Phase::FINAL_PITCH);
    CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
    CHECK(near(output.extension_m, extension));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    CHECK(near(output.pitch_rad, 20.0 * kDeg));
  }
}

void testConfiguredCruisePitchReplacesOnlyThePostClearanceTarget() {
  auto config = fastFixtureConfig();
  config.cruise_pitch_rad = 57.0 * kDeg;
  Harness harness(config);
  auto output = harness.motion();
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  harness.input.extension_m = 0.500001;
  harness.input.pitch_rad = 61.0 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::SLOW_RETRACT);
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  harness.input.extension_m = 0.50;
  output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, config.cruise_pitch_rad));
  CHECK(near(output.zipper_motor_velocity_rad_s, -config.fast_velocity_rad_s));
  harness.input.pitch_rad = 58.0 * kDeg;
  harness.input.dock_fresh = false;
  output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, config.cruise_pitch_rad, harness.initial_yaw);
  harness.input.dock_fresh = true;
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, config.cruise_pitch_rad));
}

void testHeartbeatRecoveryAndRetryPreserveClearance() {
  Harness harness;
  harness.motion();
  harness.input.extension_m = 0.53;
  harness.input.pitch_rad = 54.0 * kDeg;
  harness.input.yaw_rad = 0.25;
  harness.input.dock_fresh = false;
  auto output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, harness.initial_pitch, harness.initial_yaw);
  CHECK(output.dock_command == DockCommand::NONE);
  harness.input.dock_fresh = true;
  // Reusing the state present at interruption cannot restart the zipper.
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::SLOW_RETRACT && output.dock_retry_attempt == 0);
  CHECK(near(output.pitch_rad, harness.initial_pitch));
  harness.input.extension_m = 0.50;
  output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  harness.input.extension_m = 0.40;
  harness.input.dock_latch_open = false;
  output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_retry_attempt == 1);
  CHECK(output.dock_command == DockCommand::IDLE);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  output = harness.cycleToMotion();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.yaw_rad, harness.initial_yaw));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
}

void testPitchLimitStopsRetractionAndPreservesCruisePitch() {
  Harness harness;
  harness.motion();
  harness.input.extension_m = 0.50;
  harness.input.pitch_rad = 64.0 * kDeg;
  auto output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  // The fixed cruise target corrects below the ceiling before retracting again.
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  CHECK(output.dock_command == DockCommand::NONE);

  harness.input.pitch_rad = 62.5 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(output.actuation == Actuation::RETRACT_HOLD_PITCH);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  harness.input.pitch_rad = 64.0 * kDeg;
  harness.input.dock_fresh = false;
  output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 62.5 * kDeg;
  harness.input.dock_fresh = true;
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  harness.input.pitch_rad = 60.0 * kDeg;
  output = harness.step();
  CHECK(near(output.pitch_rad, 60.0 * kDeg));

  // The guard stays active during the later 20-degree pitch adjustment.
  harness.input.extension_m = 0.30;
  harness.input.pitch_rad = 64.0 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::STOW_RETRACT);
  checkHold(output, 20.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 62.0 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::STOW_RETRACT);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  harness.input.extension_m = 0.104;
  harness.input.pitch_rad = 64.0 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::FINAL_PITCH);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  CHECK(near(output.yaw_rad, harness.initial_yaw));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  harness.input.pitch_rad = 20.0 * kDeg;
  CHECK(harness.step().phase == Phase::COMPLETE);
}

void testPitchCeilingAlsoAppliesWhenFinalTargetIsAtLimit() {
  Controller::Config config;
  config.target_pitch_rad = 63.0 * kDeg;
  Harness harness(config);
  harness.motion();
  harness.input.extension_m = 0.104;
  harness.input.pitch_rad = 63.5 * kDeg;
  auto output = harness.step();
  // Final pitch tolerance must not allow READY while above the ceiling.
  CHECK(output.phase == Phase::FINAL_PITCH);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(near(output.pitch_rad, 62.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  harness.input.pitch_rad = 62.5 * kDeg;
  output = harness.step();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(near(output.pitch_rad, 63.0 * kDeg));
  harness.input.pitch_rad = 64.0 * kDeg;
  harness.input.dock_fresh = false;
  output = harness.step();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::HOLD_FINAL_POSE);
  CHECK(near(output.pitch_rad, 62.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
}

void testRecoveryPreservesCruisePitchAndStowProgress() {
  Harness harness;
  harness.motion();
  harness.input.extension_m = 0.50;
  harness.input.pitch_rad = 61.0 * kDeg;
  CHECK(harness.step().phase == Phase::FAST_RETRACT);
  harness.input.extension_m = 0.40;
  harness.input.pitch_rad = 64.0 * kDeg;
  harness.input.dock_fresh = false;
  auto output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 60.0 * kDeg;
  harness.input.dock_fresh = true;
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::FAST_RETRACT);
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));

  harness.input.extension_m = 0.30;
  output = harness.step();
  CHECK(output.phase == Phase::STOW_RETRACT);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  harness.input.extension_m = 0.22;
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE);
  checkHold(output, 20.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 40.0 * kDeg;
  output = harness.cycleToMotion();
  CHECK(output.phase == Phase::STOW_RETRACT);
  CHECK(output.dock_retry_attempt == 1);
  CHECK(near(output.pitch_rad, 20.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
}

void testClearanceMilestoneAndPortLossInSameUpdate() {
  Harness harness({}, 0.65);
  harness.motion();
  harness.input.extension_m = 0.55;
  harness.input.pitch_rad = 64.0 * kDeg;
  harness.input.dock_fresh = false;
  auto output = harness.step();
  CHECK(output.phase == Phase::DOCK_RECOVER);
  checkHold(output, 60.0 * kDeg, harness.initial_yaw);
  harness.input.pitch_rad = 61.0 * kDeg;
  harness.input.dock_fresh = true;
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::FAST_RETRACT);
  // Reaching clearance selects cruise pitch even when port feedback also drops.
  CHECK(near(output.pitch_rad, 60.0 * kDeg));
  CHECK(near(output.zipper_motor_velocity_rad_s, -2.0));
  CHECK(near(output.extension_m, 0.30));
}

void testMotionStopsForEachReadinessLoss() {
  for (int missing = 0; missing < 6; ++missing) {
    Harness harness;
    harness.motion();
    if (missing == 0) harness.input.dock_fresh = false;
    if (missing == 1) harness.input.dock_active = false;
    if (missing == 2) harness.input.dock_relay_active = false;
    if (missing == 3) harness.input.dock_latch_open = false;
    if (missing == 4) harness.input.dock_closing = true;
    if (missing == 5) harness.input.dock_complete = true;
    auto output = harness.step();
    CHECK(output.phase == Phase::DOCK_RECOVER);
    CHECK(output.phase != Phase::COMPLETE);
    checkHold(output, harness.initial_pitch, harness.initial_yaw);
    CHECK(output.dock_command == DockCommand::NONE);
    harness.readyPort();
    output = harness.step(0.01, true);
    CHECK(output.phase == Phase::SLOW_RETRACT && output.dock_retry_attempt == 0);
  }
}

void testBoundedCyclesAndReset() {
  Harness harness;
  harness.motion();
  for (int attempt = 1; attempt <= 4; ++attempt) {
    harness.input.dock_latch_open = false;
    auto output = harness.step();
    CHECK(output.phase == Phase::DOCK_RECOVER);
    output = harness.step(0.01, true);
    CHECK(output.phase == Phase::DOCK_DEACTIVATE);
    CHECK(output.dock_retry_attempt == attempt);
    CHECK(output.dock_command == DockCommand::IDLE);
    CHECK(harness.step().dock_command == DockCommand::NONE);
    output = harness.cycleToMotion();
    CHECK(output.phase == Phase::SLOW_RETRACT);
    CHECK(output.dock_retry_attempt == attempt);
    CHECK(near(output.pitch_rad, harness.initial_pitch));
  }
  // A transport-only recovery does not require an additional reset budget.
  harness.input.dock_fresh = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  harness.input.dock_fresh = true;
  auto output = harness.step(0.01, true);
  CHECK(output.phase == Phase::SLOW_RETRACT && output.dock_retry_attempt == 4);
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  output = harness.step(0.01, true);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP && output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 4);
  CHECK(output.status.find("exhausted 4") != std::string::npos);
  harness.controller.reset();
  output = harness.step();
  CHECK(output.phase == Phase::IDLE && output.actuation == Actuation::STOP);
  CHECK(output.dock_retry_attempt == 0);
  harness.controller.start(harness.input);
  output = harness.step();
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::IDLE);
  CHECK(output.dock_retry_attempt == 0);
}

void testPreparationAndMissingFeedbackTimeouts() {
  Harness harness;
  harness.begin();
  harness.deactivateToActivate();
  auto output = harness.step(harness.config.dock_prepare_timeout_sec + 0.001);
  CHECK(output.phase == Phase::DOCK_RECOVER);
  CHECK(output.dock_command == DockCommand::NONE);
  checkHold(output, harness.initial_pitch, harness.initial_yaw);
  harness.input.dock_fresh = false;
  output = harness.step(harness.config.dock_prepare_timeout_sec + 0.001);
  CHECK(output.phase == Phase::FAILED && output.actuation == Actuation::STOP);
  CHECK(output.dock_retry_attempt == 0);

  Controller::Config disabled;
  disabled.dock_retry_attempts = 0;
  Harness noRetries(disabled);
  noRetries.begin();
  CHECK(noRetries.step(disabled.dock_prepare_timeout_sec + 0.001).phase == Phase::DOCK_RECOVER);
  noRetries.idlePort();
  output = noRetries.step(0.01, true);
  CHECK(output.phase == Phase::FAILED && output.dock_retry_attempt == 0);
  CHECK(output.dock_command == DockCommand::NONE);
}

void testValidationAndNoApproachExtensionLimit() {
  for (int invalid = 0; invalid < 18; ++invalid) {
    Controller::Config config;
    if (invalid == 0) config.target_extension_m = -0.01;
    if (invalid == 1) config.target_pitch_rad = 90.0 * kDeg;
    if (invalid == 2) config.slow_clearance_m = std::numeric_limits<double>::quiet_NaN();
    if (invalid == 3) config.fast_velocity_rad_s = 2.1;
    if (invalid == 4) config.slow_velocity_rad_s = 2.1;
    if (invalid == 5) config.pitch_tolerance_rad = 0.0;
    if (invalid == 6) config.dock_prepare_timeout_sec = config.dock_activate_settle_sec;
    if (invalid == 7) config.dock_retry_attempts = -1;
    if (invalid == 8) config.extension_tolerance_m = config.target_extension_m + 0.01;
    if (invalid == 9) config.intermediate_extension_m = config.target_extension_m;
    if (invalid == 10) config.intermediate_extension_m =
        config.target_extension_m + config.extension_tolerance_m;
    if (invalid == 11) config.intermediate_extension_m =
        std::numeric_limits<double>::quiet_NaN();
    if (invalid == 12) config.pitch_max_rad = 64.0 * kDeg;
    if (invalid == 13) {
      config.pitch_min_rad = 62.5 * kDeg;
      config.target_pitch_rad = 63.0 * kDeg;
      config.cruise_pitch_rad = 63.0 * kDeg;
    }
    if (invalid == 14) config.cruise_pitch_rad =
        std::numeric_limits<double>::quiet_NaN();
    if (invalid == 15) config.cruise_pitch_rad =
        std::numeric_limits<double>::infinity();
    if (invalid == 16) config.cruise_pitch_rad = config.pitch_min_rad - kDeg;
    if (invalid == 17) config.cruise_pitch_rad = config.pitch_max_rad + kDeg;
    Controller controller(config);
    Controller::Input input;
    input.extension_m = 0.60;
    input.pitch_rad = (invalid == 13 ? 62.75 : 55.0) * kDeg;
    controller.start(input);
    const auto output = controller.update(input);
    CHECK(output.phase == Phase::FAILED && output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
  }
  for (int invalid = 0; invalid < 4; ++invalid) {
    Controller::Config config;
    Controller controller(config);
    Controller::Input input;
    input.extension_m = 0.6;
    input.pitch_rad = 55.0 * kDeg;
    if (invalid == 0) input.extension_m = -0.01;
    if (invalid == 1) input.pitch_rad = 64.0 * kDeg;
    if (invalid == 2) input.yaw_rad = std::numeric_limits<double>::quiet_NaN();
    if (invalid == 3) input.now_sec = std::numeric_limits<double>::quiet_NaN();
    controller.start(input);
    CHECK(controller.update(input).phase == Phase::FAILED);
  }
  Harness longExtension({}, 1.2);
  CHECK(longExtension.begin().phase == Phase::DOCK_DEACTIVATE);
  longExtension.input.now_sec = -1.0;
  const auto output = longExtension.controller.update(longExtension.input);
  CHECK(output.phase == Phase::FAILED && output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
}
}  // namespace

int main() {
  try {
    testProductionCommandDwellBeforeUndocking();
    testUndockingCommandsAndRetryStaySpaced();
    testQueuedUndockingStartsTimeoutAtEmission();
    testUndockingCommandIntervalValidation();
    testCommandOrderAndHeldButton();
    testMeasuredClearanceAndFinalPose();
    testShortStartingExtensionNeverExtendsOrOverRetracts();
    testConfiguredCruisePitchReplacesOnlyThePostClearanceTarget();
    testHeartbeatRecoveryAndRetryPreserveClearance();
    testPitchLimitStopsRetractionAndPreservesCruisePitch();
    testPitchCeilingAlsoAppliesWhenFinalTargetIsAtLimit();
    testRecoveryPreservesCruisePitchAndStowProgress();
    testClearanceMilestoneAndPortLossInSameUpdate();
    testMotionStopsForEachReadinessLoss();
    testBoundedCyclesAndReset();
    testPreparationAndMissingFeedbackTimeouts();
    testValidationAndNoApproachExtensionLimit();
    std::cout << "Autonomous undocking tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
