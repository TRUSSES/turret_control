#include "autonomous_docking.h"
#include "docking_yaw_control.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::AutonomousDockingController;
using Phase = Controller::Phase;
using Actuation = Controller::Actuation;
using DockCommand = Controller::DockCommand;
using YawController = turret_control::DockingYawController;
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
  config.dock_open_settle_sec = config.latch_close_settle_sec = 0.6;
  return config;
}

Controller::Config fixtureConfig(Controller::Config settings, bool use_command_delay) {
  // Existing scenarios isolate their original state/settle behavior. The delay
  // scenarios opt in explicitly and preserve production or custom spacing.
  if (!use_command_delay) settings.dock_command_interval_sec = 0.0;
  return settings;
}

struct Harness {
  Controller::Config config;
  Controller controller;
  Controller::Input input;

  explicit Harness(Controller::Config settings = fastFixtureConfig(),
                   bool use_command_delay = false)
      : config(fixtureConfig(settings, use_command_delay)), controller(config) {
    input.extension_m = 0.25;
    input.pitch_rad = 20.0 * kDeg;
    input.yaw_rad = 0.1;
    input.dock_fresh = true;
    input.dock_idle = input.dock_unoccupied = true;
    input.dock_state_sequence = 1;
    input.camera_y_m = settings.goal_camera_y_m;
    input.camera_z_m = 0.5;
    input.camera_now_ns = 1000000000;
    input.pose_stamp_ns = input.camera_now_ns;
    controller.start(input);
  }

  Controller::Output step(double dt = 0.1, bool new_capture = true) {
    input.now_sec += dt;
    input.camera_now_ns += static_cast<std::int64_t>(dt * 1e9);
    if (new_capture) input.pose_stamp_ns = input.camera_now_ns;
    return controller.update(input);
  }

  void follow(const Controller::Output& output) {
    input.extension_m = output.extension_m;
    input.pitch_rad = output.pitch_rad;
    input.yaw_rad = output.yaw_rad;
  }

  Controller::Output approach() {
    input.pose_fresh = true;
    input.camera_x_m = config.goal_camera_x_m;
    input.camera_y_m = config.goal_camera_y_m;
    return step();
  }

  Controller::Output prepare() {
    approach();
    input.camera_z_m = config.insertion_distance_m;
    return step();
  }

  Controller::Output activateToOpen() {
    CHECK(controller.phase() == Phase::DOCK_ACTIVATE);
    input.dock_idle = false;
    input.dock_unoccupied = true;
    input.dock_active = input.dock_relay_active = true;
    input.dock_latch_open = false;
    auto output = step(config.dock_activate_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_ACTIVATE);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::DOCK_OPEN);
    CHECK(output.dock_command == DockCommand::OPEN_LATCH);
    return output;
  }

  Controller::Output openToInsert() {
    CHECK(controller.phase() == Phase::DOCK_OPEN);
    input.dock_latch_open = true;
    auto output = step(config.dock_open_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_OPEN);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.dock_command == DockCommand::NONE);
    return output;
  }

  void idleDock() {
    input.dock_fresh = input.dock_idle = input.dock_unoccupied = true;
    input.dock_active = input.dock_relay_active = false;
    input.dock_latch_open = false;
  }

  Controller::Output recoverToDeactivate() {
    CHECK(controller.phase() == Phase::DOCK_RECOVER);
    idleDock();
    ++input.dock_state_sequence;
    const auto output = step(0.01);
    CHECK(output.phase == Phase::DOCK_DEACTIVATE);
    CHECK(output.dock_command == DockCommand::IDLE);
    return output;
  }

  Controller::Output deactivateToActivate() {
    CHECK(controller.phase() == Phase::DOCK_DEACTIVATE);
    idleDock();
    auto output = step(config.dock_deactivate_settle_sec + 0.001);
    CHECK(output.phase == Phase::DOCK_DEACTIVATE);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::DOCK_ACTIVATE);
    CHECK(output.dock_command == DockCommand::ACTIVATE);
    return output;
  }

  Controller::Output finishPreparation() {
    activateToOpen();
    return openToInsert();
  }

  Controller::Output insert() {
    auto output = prepare();
    CHECK(output.phase == Phase::DOCK_ACTIVATE);
    CHECK(output.dock_command == DockCommand::ACTIVATE);
    return finishPreparation();
  }

  Controller::Output latchDeactivateToActivate() {
    CHECK(controller.phase() == Phase::LATCH_DEACTIVATE);
    input.dock_fresh = input.dock_idle = true;
    input.dock_active = input.dock_relay_active = false;
    input.dock_latch_open = input.dock_complete = input.dock_closing = false;
    input.dock_unoccupied = !input.dock_detected;
    input.dock_latch_pulse_width_us = 0;
    auto output = step(config.dock_deactivate_settle_sec + 0.001);
    CHECK(output.phase == Phase::LATCH_DEACTIVATE);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::LATCH_ACTIVATE);
    CHECK(output.dock_command == DockCommand::ACTIVATE);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    return output;
  }

  Controller::Output latchActivateToClose() {
    CHECK(controller.phase() == Phase::LATCH_ACTIVATE);
    input.dock_idle = false;
    input.dock_active = input.dock_relay_active = true;
    input.dock_latch_open = false;
    input.dock_latch_pulse_width_us = config.latch_closed_pulse_width_us;
    auto output = step(config.dock_activate_settle_sec + 0.001);
    CHECK(output.phase == Phase::LATCH_ACTIVATE);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::WAIT_FOR_LATCH);
    CHECK(output.dock_command == DockCommand::CLOSE_LATCH);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    return output;
  }

  Controller::Output closeToComplete() {
    CHECK(controller.phase() == Phase::WAIT_FOR_LATCH);
    input.dock_fresh = input.dock_active = input.dock_relay_active = true;
    input.dock_latch_open = false;
    input.dock_latch_pulse_width_us = config.latch_closed_pulse_width_us;
    auto output = step(config.latch_close_settle_sec + 0.001);
    CHECK(output.phase == Phase::WAIT_FOR_LATCH);
    CHECK(output.dock_command == DockCommand::NONE);
    ++input.dock_state_sequence;
    output = step(0.01);
    CHECK(output.phase == Phase::COMPLETE);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
    return output;
  }

  Controller::Output lose() {
    approach();
    input.pose_fresh = false;
    return step();
  }

  void reach(Phase phase) {
    if (phase == Phase::SEARCH) return;
    if (phase == Phase::PITCH_CENTER) {
      input.pose_fresh = true;
      input.camera_y_m = config.goal_camera_y_m + 0.08;
      step();
      return;
    }
    if (phase == Phase::APPROACH) { approach(); return; }
    if (phase == Phase::WAIT_FOR_TAG) { lose(); return; }
    if (phase == Phase::DOCK_ACTIVATE) { prepare(); return; }
    if (phase == Phase::DOCK_OPEN) { prepare(); activateToOpen(); return; }
    if (phase == Phase::DOCK_RECOVER || phase == Phase::DOCK_DEACTIVATE) {
      insert();
      input.dock_fresh = false;
      CHECK(step().phase == Phase::DOCK_RECOVER);
      if (phase == Phase::DOCK_DEACTIVATE) recoverToDeactivate();
      return;
    }
    if (phase == Phase::INSERT) { insert(); return; }
    if (phase == Phase::LATCH_DEACTIVATE || phase == Phase::LATCH_ACTIVATE ||
        phase == Phase::WAIT_FOR_LATCH || phase == Phase::LATCH_RECOVER) {
      insert();
      input.dock_detected = true;
      input.dock_unoccupied = false;
      CHECK(step().phase == Phase::LATCH_DEACTIVATE);
      if (phase == Phase::LATCH_DEACTIVATE) return;
      latchDeactivateToActivate();
      if (phase == Phase::LATCH_ACTIVATE) return;
      latchActivateToClose();
      if (phase == Phase::LATCH_RECOVER) {
        input.dock_fresh = false;
        CHECK(step(config.latch_timeout_sec + 0.001).phase == Phase::LATCH_RECOVER);
      }
    }
  }
};

struct YawHarness {
  YawController::Config config;
  YawController controller;
  YawController::Input input;

  explicit YawHarness(YawController::Config settings = YawController::Config{},
                      double initial_yaw = 0.0)
      : config(settings), controller(settings) {
    input.camera_now_ns = input.pose_stamp_ns = 1000000000;
    input.yaw_rad = initial_yaw;
    input.requested_yaw_rad = initial_yaw + 2.0 * kDeg;
    input.camera_x_error_m = 0.05;
    input.allow_alignment = true;
    input.feedback_sequence = 1;
    input.require_recovery_capture = true;
    controller.start(0.0, initial_yaw, input.feedback_sequence);
  }

  void advance(double dt, bool new_capture, bool new_feedback) {
    input.now_sec += dt;
    input.camera_now_ns += static_cast<std::int64_t>(dt * 1e9);
    if (new_capture) input.pose_stamp_ns = input.camera_now_ns;
    if (new_feedback) ++input.feedback_sequence;
  }

  YawController::Output observe(double dt = 0.01, bool new_capture = true,
                                bool new_feedback = true) {
    advance(dt, new_capture, new_feedback);
    const auto output = controller.observeFeedback(input);
    if (output.feedback_recovery_required || output.failed) {
      CHECK(near(output.motor_velocity_rad_s, 0.0));
      CHECK(!output.adjusting);
    }
    return output;
  }

  YawController::Output step(double dt = 0.01, bool new_capture = true,
                            bool new_feedback = true) {
    advance(dt, new_capture, new_feedback);
    const auto output = controller.update(input);
    // Only an unfinished correction authorizes a motor velocity pulse. Every
    // stopped, canceled, observed, completed, or failed result must remove it.
    if (output.adjusting && !output.failed && !output.feedback_recovery_required) {
      const double sign = output.yaw_rad > input.yaw_rad ? 1.0 : -1.0;
      CHECK(near(output.motor_velocity_rad_s, sign * config.motor_velocity_rad_s));
    } else {
      CHECK(near(output.motor_velocity_rad_s, 0.0));
    }
    return output;
  }
};

void testSearch() {
  Harness harness;
  CHECK(harness.controller.phase() == Phase::SEARCH);
  auto output = harness.step();
  CHECK(output.actuation == Actuation::TRACK_POSE);
  CHECK(output.pitch_rad > harness.input.pitch_rad);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));

  Controller::Config reverse;
  reverse.pitch_down_sign = -1.0;
  Harness reversed(reverse);
  CHECK(reversed.step().pitch_rad < reversed.input.pitch_rad);

  // Commands at 100 Hz must cross the physical pitch controller's deadband
  // without accumulating an unbounded lead over a motor that has not moved.
  Harness fast;
  for (int index = 0; index < 20; ++index) output = fast.step(0.01);
  CHECK(output.pitch_rad - fast.input.pitch_rad > 0.5 * kDeg);
  CHECK(output.pitch_rad - fast.input.pitch_rad <= fast.config.max_pitch_step_rad + 1e-9);
  for (int index = 0; index < 1000; ++index) output = fast.step(0.01);
  CHECK(output.pitch_rad - fast.input.pitch_rad <= fast.config.max_pitch_step_rad + 1e-9);
  CHECK(near(output.extension_m, fast.input.extension_m));

  Harness delayed;
  output = delayed.step(100.0);
  CHECK(output.pitch_rad - delayed.input.pitch_rad <=
        delayed.config.search_pitch_rate_rad_s * delayed.config.max_dt_sec + 1e-9);
  CHECK(delayed.step(100.0).phase == Phase::SEARCH);

  Controller::Config smallSpan;
  smallSpan.search_pitch_span_rad = 1.0 * kDeg;
  Harness bounded(smallSpan);
  for (int index = 0; index < 30; ++index) {
    output = bounded.step();
    bounded.follow(output);
  }
  CHECK(near(bounded.input.pitch_rad, 21.0 * kDeg));
  CHECK(near(bounded.input.extension_m, 0.25));
  CHECK(bounded.step(100.0).phase == Phase::SEARCH);
}

void testPitchCenteringAndHolding() {
  Harness harness;
  harness.input.pose_fresh = true;
  harness.input.camera_x_m = 0.04;
  harness.input.camera_y_m = 0.08;
  auto output = harness.step();
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(output.yaw_rad > harness.input.yaw_rad);
  CHECK(output.pitch_rad < harness.input.pitch_rad);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  harness.follow(output);

  // Extension starts only after the tag reaches the inner angular window.
  harness.input.camera_y_m = harness.config.goal_camera_y_m + 0.005;
  const double centeredPitch = harness.input.pitch_rad;
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.pitch_rad, centeredPitch));
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.5));

  // Camera motion inside the outer window does not change the held pitch.
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(3.0 * kDeg);
  harness.input.pitch_rad += 0.003;
  CHECK(!harness.controller.needsPitchCentering(harness.input));
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.pitch_rad, centeredPitch));

  // Crossing the outer window stops the active extension burst immediately.
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(4.1 * kDeg);
  CHECK(harness.controller.needsPitchCentering(harness.input));
  output = harness.step();
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(output.pitch_rad < harness.input.pitch_rad);

  // Hysteresis requires a return inside 2 degrees, not merely below 4.
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(3.0 * kDeg);
  CHECK(harness.step().phase == Phase::PITCH_CENTER);
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(0.5 * kDeg);
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.pitch_rad, harness.input.pitch_rad));
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);

  Controller::Config reverse;
  reverse.yaw_correction_sign = -1.0;
  Harness reversed(reverse);
  reversed.input.pose_fresh = true;
  reversed.input.camera_x_m = 0.04;
  CHECK(reversed.step().yaw_rad < reversed.input.yaw_rad);

  Harness deadband;
  deadband.input.pose_fresh = true;
  deadband.input.camera_x_m = 0.003;
  deadband.input.camera_y_m = deadband.config.goal_camera_y_m + 0.003;
  output = deadband.step();
  CHECK(near(output.yaw_rad, deadband.input.yaw_rad));
  CHECK(near(output.pitch_rad, deadband.input.pitch_rad));
  CHECK(near(output.extension_m, deadband.input.extension_m));
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);

  Harness clamped;
  clamped.input.pitch_rad = clamped.config.pitch_max_rad - 0.005;
  clamped.controller.start(clamped.input);
  clamped.input.pose_fresh = true;
  clamped.input.camera_x_m = 1.0;
  clamped.input.camera_y_m = -1.0;
  const double startingYaw = clamped.input.yaw_rad;
  for (int index = 0; index < 100; ++index) {
    output = clamped.step();
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(near(output.extension_m, clamped.input.extension_m));
    CHECK(output.pitch_rad <= clamped.config.pitch_max_rad + 1e-9);
    CHECK(std::fabs(output.yaw_rad - clamped.input.yaw_rad) <=
          clamped.config.max_yaw_rate_rad_s * 0.1 + 1e-9);
    clamped.follow(output);
  }
  CHECK(near(output.pitch_rad, clamped.config.pitch_max_rad));
  // Pitch remains mechanically bounded while valid visual yaw corrections
  // continue beyond the former 30-degree displacement from docking start.
  CHECK(output.yaw_rad > startingYaw + 30.0 * kDeg);

  // Just outside the handoff, 8 mm exceeds the 2 degree window.
  // Center pitch with the zipper stopped.
  Harness closeCenter;
  closeCenter.input.pose_fresh = true;
  closeCenter.input.camera_z_m = closeCenter.config.insertion_distance_m + 0.01;
  closeCenter.input.camera_y_m = closeCenter.config.goal_camera_y_m + 0.008;
  output = closeCenter.step();
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(output.pitch_rad < closeCenter.input.pitch_rad);
  CHECK(near(output.extension_m, closeCenter.input.extension_m));
}

void setCameraPitchError(Harness& harness, double error_rad) {
  harness.input.pose_fresh = true;
  harness.input.camera_y_m = harness.config.goal_camera_y_m -
      harness.input.camera_z_m * std::tan(error_rad);
}

void testAdaptivePitchCoarseFineAndDirections() {
  // A large image error advances the pitch target three times faster than a
  // small error; both directions stop the zipper until the tag is centered.
  for (double direction : {-1.0, 1.0}) {
    Harness coarse;
    setCameraPitchError(coarse, direction * 10.0 * kDeg);
    auto output = coarse.step();
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(output.actuation == Actuation::CENTER_PITCH);
    CHECK(near(output.pitch_rad - coarse.input.pitch_rad, direction * 1.5 * kDeg));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));

    Harness fine;
    setCameraPitchError(fine, direction * 2.5 * kDeg);
    output = fine.step();
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(near(output.pitch_rad - fine.input.pitch_rad, direction * 0.5 * kDeg));
    for (int tick = 0; tick < 40; ++tick) output = fine.step();
    CHECK(near(output.pitch_rad - fine.input.pitch_rad, direction * 2.0 * kDeg));

    // A blocked motor cannot accumulate a large latent correction. Search
    // and coarse camera correction allow six degrees, fine correction two.
    for (int tick = 0; tick < 40; ++tick) output = coarse.step();
    CHECK(near(output.pitch_rad - coarse.input.pitch_rad, direction * 6.0 * kDeg));
    CHECK(near(output.extension_m, coarse.input.extension_m));
  }

  Harness search;
  auto output = search.step();
  CHECK(near(output.pitch_rad - search.input.pitch_rad, 1.5 * kDeg));
  for (int tick = 0; tick < 40; ++tick) output = search.step();
  CHECK(near(output.pitch_rad - search.input.pitch_rad, 6.0 * kDeg));

  Harness approach;
  CHECK(near(approach.approach().zipper_motor_velocity_rad_s, 1.5));
  CHECK(near(approach.insert().zipper_motor_velocity_rad_s, 1.4));
}

double firstPitchAdvance(double camera_error_rad) {
  Harness harness;
  setCameraPitchError(harness, camera_error_rad);
  return harness.step().pitch_rad - harness.input.pitch_rad;
}

void testAdaptivePitchSmoothTransition() {
  // The transition is continuous at both ends, and its midpoint is faster
  // than fine correction while remaining slower than coarse correction.
  const double fine = firstPitchAdvance(3.0 * kDeg);
  const double coarse = firstPitchAdvance(8.0 * kDeg);
  const double middle = firstPitchAdvance(5.5 * kDeg);
  CHECK(near(fine, 0.5 * kDeg));
  CHECK(near(coarse, 1.5 * kDeg));
  CHECK(middle > fine && middle < coarse);
  CHECK(near(firstPitchAdvance(-5.5 * kDeg), -middle));
  for (double threshold : {3.0 * kDeg, 8.0 * kDeg}) {
    const double below = firstPitchAdvance(threshold - 0.01 * kDeg);
    const double above = firstPitchAdvance(threshold + 0.01 * kDeg);
    CHECK(std::fabs(above - below) < 0.002 * kDeg);
    CHECK(above >= below);
  }

  Harness delayed;
  setCameraPitchError(delayed, 10.0 * kDeg);
  const auto output = delayed.step(10.0);
  CHECK(near(output.pitch_rad - delayed.input.pitch_rad, 1.5 * kDeg));
}

void testAdaptivePitchRemainingErrorAndReversal() {
  // Use a deliberately tighter local acceptance window to exercise the
  // remaining-error clamp below the production 2 degree centering tolerance.
  Controller::Config precise;
  precise.centering_start_tolerance_rad = 1.0 * kDeg;
  for (double direction : {-1.0, 1.0}) {
    Harness close(precise);
    setCameraPitchError(close, direction * 1.1 * kDeg);
    Controller::Output output;
    for (int tick = 0; tick < 20; ++tick) {
      output = close.step();
      CHECK(direction * (output.pitch_rad - close.input.pitch_rad) > 0.0);
      CHECK(direction * (output.pitch_rad - close.input.pitch_rad) <= 1.1 * kDeg + 1e-9);
    }
    CHECK(near(output.pitch_rad - close.input.pitch_rad, direction * 1.1 * kDeg));

    Harness changing(precise);
    setCameraPitchError(changing, direction * 10.0 * kDeg);
    for (int tick = 0; tick < 20; ++tick) output = changing.step();
    CHECK(near(output.pitch_rad - changing.input.pitch_rad, direction * 6.0 * kDeg));
    // A newer image reduces the remaining correction below the accumulated
    // coarse lead. The target must stay within what this image still needs.
    setCameraPitchError(changing, direction * 1.2 * kDeg);
    output = changing.step();
    CHECK(near(output.pitch_rad - changing.input.pitch_rad, direction * 1.2 * kDeg));
    // Reversing the camera error must discard the old direction immediately.
    setCameraPitchError(changing, -direction * 2.0 * kDeg);
    output = changing.step();
    CHECK(near(output.pitch_rad - changing.input.pitch_rad, -direction * 0.5 * kDeg));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  }

  for (double direction : {-1.0, 1.0}) {
    Harness bound;
    bound.input.pitch_rad = direction > 0.0
        ? bound.config.pitch_max_rad - 0.25 * kDeg
        : bound.config.pitch_min_rad + 0.25 * kDeg;
    bound.controller.start(bound.input);
    setCameraPitchError(bound, direction * 10.0 * kDeg);
    for (int tick = 0; tick < 20; ++tick) {
      const auto output = bound.step();
      CHECK(output.pitch_rad >= bound.config.pitch_min_rad);
      CHECK(output.pitch_rad <= bound.config.pitch_max_rad);
      CHECK(near(output.pitch_rad, direction > 0.0
          ? bound.config.pitch_max_rad : bound.config.pitch_min_rad));
    }
  }
}

void testAdaptivePitchConfiguration() {
  using Field = double Controller::Config::*;
  const Field positive_fields[] = {
    &Controller::Config::fine_pitch_rate_rad_s,
    &Controller::Config::fine_pitch_step_rad,
    &Controller::Config::pitch_fine_error_rad,
    &Controller::Config::pitch_coarse_error_rad
  };
  for (Field field : positive_fields) {
    for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
      Controller::Config config;
      config.*field = value;
      Harness invalid(config);
      CHECK(invalid.step().phase == Phase::FAILED);
    }
  }
  for (int invalid_setting = 0; invalid_setting < 4; ++invalid_setting) {
    Controller::Config config;
    if (invalid_setting == 0) config.fine_pitch_rate_rad_s = config.max_pitch_rate_rad_s + kDeg;
    if (invalid_setting == 1) config.fine_pitch_step_rad = config.max_pitch_step_rad + kDeg;
    if (invalid_setting == 2) config.pitch_fine_error_rad = config.centering_start_tolerance_rad;
    if (invalid_setting == 3) config.pitch_coarse_error_rad = config.pitch_fine_error_rad;
    Harness invalid(config);
    CHECK(invalid.step().phase == Phase::FAILED);
  }
  Controller::Config constant;
  constant.fine_pitch_rate_rad_s = constant.max_pitch_rate_rad_s;
  constant.fine_pitch_step_rad = constant.max_pitch_step_rad;
  Harness valid(constant);
  CHECK(valid.step().phase == Phase::SEARCH);
}

void testInsertionHandoff() {
  // The default handoff is 20 cm. An aligned tag at 21 cm still needs an
  // approach burst; at 20 cm dock preparation holds pitch/yaw before insertion.
  Harness threshold;
  CHECK(near(threshold.config.insertion_distance_m, 0.20));
  threshold.approach();
  threshold.input.camera_z_m = 0.21;
  auto output = threshold.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  threshold.input.camera_z_m = threshold.config.insertion_distance_m;
  output = threshold.step();
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(near(output.handoff_camera_z_m, threshold.config.insertion_distance_m));
  output = threshold.finishPreparation();
  CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);

  Harness harness;
  output = harness.approach();
  CHECK(output.phase == Phase::APPROACH);
  harness.follow(output);
  harness.input.camera_z_m = harness.config.insertion_distance_m + 0.01;
  CHECK(harness.step().phase == Phase::APPROACH);

  harness.input.camera_z_m = harness.config.insertion_distance_m;
  harness.input.camera_x_m = 0.02;
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(3.0 * kDeg);
  CHECK(harness.controller.needsInsertionUpdate(harness.input));
  output = harness.step();
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(near(output.pitch_rad, harness.input.pitch_rad));
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  output = harness.finishPreparation();
  CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.4));

  Harness missingDock;
  missingDock.approach();
  missingDock.input.camera_z_m = missingDock.config.insertion_distance_m;
  missingDock.input.dock_fresh = false;
  output = missingDock.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));

  Harness maxExtension;
  maxExtension.approach();
  maxExtension.input.extension_m = maxExtension.config.extension_max_m;
  CHECK(maxExtension.controller.needsApproachUpdate(maxExtension.input));
  output = maxExtension.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness fast;
  fast.approach();
  for (int index = 0; index < 30; ++index) output = fast.step(0.01);
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, fast.config.zipper_motor_velocity_rad_s));
  CHECK(near(output.extension_m, fast.input.extension_m));

  Harness depthDeadband;
  depthDeadband.approach();
  depthDeadband.input.camera_z_m = depthDeadband.config.insertion_distance_m +
      0.6 * depthDeadband.config.depth_deadband_m;
  CHECK(depthDeadband.step().phase == Phase::DOCK_ACTIVATE);

  Harness outsideDepthDeadband;
  outsideDepthDeadband.approach();
  outsideDepthDeadband.input.camera_z_m = outsideDepthDeadband.config.insertion_distance_m +
      outsideDepthDeadband.config.depth_deadband_m + 0.001;
  CHECK(outsideDepthDeadband.step().phase == Phase::APPROACH);

  Harness finalCenter;
  finalCenter.approach();
  finalCenter.input.camera_z_m = finalCenter.config.insertion_distance_m;
  finalCenter.input.camera_y_m = finalCenter.config.goal_camera_y_m +
      finalCenter.input.camera_z_m * std::tan(3.0 * kDeg);
  CHECK(finalCenter.controller.needsInsertionUpdate(finalCenter.input));
  output = finalCenter.step();
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(near(output.extension_m, finalCenter.input.extension_m));
  CHECK(near(output.pitch_rad, finalCenter.input.pitch_rad));
  CHECK(near(output.yaw_rad, finalCenter.input.yaw_rad));
}

void testInsertionPriorityAndFastHook() {
  for (Phase phase : {Phase::SEARCH, Phase::PITCH_CENTER, Phase::APPROACH, Phase::WAIT_FOR_TAG}) {
    Harness harness;
    harness.reach(phase);
    CHECK(harness.controller.phase() == phase);
    harness.input.pose_fresh = true;
    harness.input.camera_z_m = harness.config.insertion_distance_m;
    harness.input.camera_x_m = harness.config.goal_camera_x_m + 0.04;
    harness.input.camera_y_m = harness.config.goal_camera_y_m +
        harness.input.camera_z_m * std::tan(5.0 * kDeg);
    CHECK(harness.controller.needsInsertionUpdate(harness.input));

    // The fast hook must never turn a stale or invalid camera pose into a
    // handoff, even when its cached depth was within the insertion distance.
    auto rejected = harness.input;
    rejected.pose_fresh = false;
    CHECK(!harness.controller.needsInsertionUpdate(rejected));
    rejected = harness.input;
    rejected.camera_x_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(!harness.controller.needsInsertionUpdate(rejected));
    rejected = harness.input;
    rejected.camera_y_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(!harness.controller.needsInsertionUpdate(rejected));
    rejected = harness.input;
    rejected.camera_z_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(!harness.controller.needsInsertionUpdate(rejected));
    rejected.camera_z_m = 0.0;
    CHECK(!harness.controller.needsInsertionUpdate(rejected));
    rejected.camera_z_m = harness.config.insertion_distance_m +
        harness.config.depth_deadband_m + 0.001;
    CHECK(!harness.controller.needsInsertionUpdate(rejected));

    auto output = harness.step();
    CHECK(output.phase == Phase::DOCK_ACTIVATE);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::ACTIVATE);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    CHECK(near(output.pitch_rad, harness.input.pitch_rad));
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));
    CHECK(!harness.controller.needsInsertionUpdate(harness.input));

    Harness missingDock;
    missingDock.reach(phase);
    missingDock.input.pose_fresh = true;
    missingDock.input.camera_z_m = missingDock.config.insertion_distance_m;
    missingDock.input.dock_fresh = false;
    output = missingDock.step();
    CHECK(output.phase == Phase::FAILED);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
  }
}

void testCloseRangeHandoffRegression() {
  // Recorded hardware pose: the tag is close enough but x=26 mm is outside
  // lateral tolerance. Capture the measured joint angles immediately.
  Harness harness;
  harness.input.extension_m = 0.208;
  harness.input.pitch_rad = 59.41 * kDeg;
  harness.input.yaw_rad = 0.27;
  harness.input.pose_fresh = true;
  harness.input.camera_z_m = 0.198;
  harness.input.camera_x_m = 0.026;
  harness.input.camera_y_m = harness.config.goal_camera_y_m -
      harness.input.camera_z_m * std::tan(-0.69 * kDeg);
  CHECK(harness.controller.needsInsertionUpdate(harness.input));
  auto output = harness.step();
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(near(output.handoff_camera_z_m, 0.198));
  CHECK(near(output.extension_m, 0.208));
  CHECK(near(output.pitch_rad, 59.41 * kDeg));
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  const double heldPitch = output.pitch_rad;
  const double heldYaw = output.yaw_rad;

  harness.input.pose_fresh = false;
  harness.input.camera_x_m = harness.input.camera_y_m = harness.input.camera_z_m =
      std::numeric_limits<double>::quiet_NaN();
  harness.input.extension_m += 0.12;
  harness.input.pitch_rad += 0.01;
  harness.input.yaw_rad += 0.01;
  output = harness.finishPreparation();
  CHECK(output.phase == Phase::INSERT);
  CHECK(near(output.pitch_rad, heldPitch));
  CHECK(near(output.yaw_rad, heldYaw));
  output = harness.step(1000.0);
  CHECK(output.phase == Phase::INSERT);
  CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);
  CHECK(near(output.pitch_rad, heldPitch));
  CHECK(near(output.yaw_rad, heldYaw));
  CHECK(output.zipper_motor_velocity_rad_s > 0.0);
  harness.input.dock_detected = true;
  output = harness.step();
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::IDLE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
}

void checkDockHold(const Controller::Output& output, Phase phase,
                   const Controller::Output& handoff,
                   DockCommand command = DockCommand::NONE) {
  CHECK(output.phase == phase);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == command);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(near(output.pitch_rad, handoff.pitch_rad));
  CHECK(near(output.yaw_rad, handoff.yaw_rad));
  CHECK(near(output.handoff_camera_z_m, handoff.handoff_camera_z_m));
}

void testDockPreparationOrderAndCapture() {
  Harness harness;
  // Existing ACTIVE/open flags are from an earlier command and cannot skip
  // the new ACTIVATE -> closed confirmation -> OPEN_LATCH -> open sequence.
  harness.input.dock_active = harness.input.dock_relay_active = true;
  harness.input.dock_latch_open = true;
  harness.input.dock_state_sequence = 10;
  const auto handoff = harness.prepare();
  checkDockHold(handoff, Phase::DOCK_ACTIVATE, handoff, DockCommand::ACTIVATE);
  CHECK(near(handoff.handoff_camera_z_m, harness.config.insertion_distance_m));
  CHECK(!harness.controller.allowsYawAlignment(harness.input));

  harness.input.pose_fresh = false;
  harness.input.camera_x_m = harness.input.camera_y_m = harness.input.camera_z_m =
      std::numeric_limits<double>::quiet_NaN();
  harness.input.pitch_rad += 3.0 * kDeg;
  harness.input.yaw_rad += 1.0 * kDeg;
  harness.input.extension_m += 0.01;
  auto output = harness.step(0.2);
  checkDockHold(output, Phase::DOCK_ACTIVATE, handoff);
  CHECK(near(output.extension_m, harness.input.extension_m));

  harness.input.dock_latch_open = false;
  ++harness.input.dock_state_sequence;  // A reply received before the dwell ends.
  output = harness.step(harness.config.dock_activate_settle_sec);
  checkDockHold(output, Phase::DOCK_ACTIVATE, handoff);
  output = harness.step();  // Correct cached flags and sequence still cannot pass.
  checkDockHold(output, Phase::DOCK_ACTIVATE, handoff);
  ++harness.input.dock_state_sequence;
  output = harness.step();
  checkDockHold(output, Phase::DOCK_OPEN, handoff, DockCommand::OPEN_LATCH);
  CHECK(!harness.controller.allowsYawAlignment(harness.input));

  harness.input.dock_latch_open = true;
  ++harness.input.dock_state_sequence;  // Even open feedback cannot skip the dwell.
  output = harness.step(0.1);
  checkDockHold(output, Phase::DOCK_OPEN, handoff);
  output = harness.step(harness.config.dock_open_settle_sec);
  checkDockHold(output, Phase::DOCK_OPEN, handoff);
  output = harness.step();
  checkDockHold(output, Phase::DOCK_OPEN, handoff);
  ++harness.input.dock_state_sequence;
  output = harness.step();
  CHECK(output.phase == Phase::INSERT);
  CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(near(output.pitch_rad, handoff.pitch_rad));
  CHECK(near(output.yaw_rad, handoff.yaw_rad));
  CHECK(near(output.handoff_camera_z_m, handoff.handoff_camera_z_m));
  CHECK(near(output.zipper_motor_velocity_rad_s, harness.config.insertion_velocity_rad_s));
  CHECK(!harness.controller.allowsYawAlignment(harness.input));

  // Preparation's timeout does not become a deadline for final insertion.
  output = harness.step(1000.0);
  CHECK(output.phase == Phase::INSERT);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(near(output.pitch_rad, handoff.pitch_rad));
  CHECK(near(output.yaw_rad, handoff.yaw_rad));
}

void testDockPreparationRequiresFlags() {
  for (int invalidFlag = 0; invalidFlag < 3; ++invalidFlag) {
    Harness harness;
    const auto handoff = harness.prepare();
    harness.step(harness.config.dock_activate_settle_sec + 0.001);
    harness.input.dock_active = invalidFlag != 0;
    harness.input.dock_relay_active = invalidFlag != 1;
    harness.input.dock_latch_open = invalidFlag == 2;
    ++harness.input.dock_state_sequence;
    auto output = harness.step();
    checkDockHold(output, Phase::DOCK_ACTIVATE, handoff);
    harness.input.dock_active = harness.input.dock_relay_active = true;
    harness.input.dock_latch_open = false;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    checkDockHold(output, Phase::DOCK_OPEN, handoff, DockCommand::OPEN_LATCH);
  }

  for (int invalidFlag = 0; invalidFlag < 3; ++invalidFlag) {
    Harness harness;
    const auto handoff = harness.prepare();
    harness.activateToOpen();
    harness.step(harness.config.dock_open_settle_sec + 0.001);
    harness.input.dock_active = invalidFlag != 0;
    harness.input.dock_relay_active = invalidFlag != 1;
    harness.input.dock_latch_open = invalidFlag != 2;
    ++harness.input.dock_state_sequence;
    auto output = harness.step();
    checkDockHold(output, Phase::DOCK_OPEN, handoff);
    harness.input.dock_active = harness.input.dock_relay_active =
        harness.input.dock_latch_open = true;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(near(output.pitch_rad, handoff.pitch_rad));
    CHECK(near(output.yaw_rad, handoff.yaw_rad));
  }
}

void testDockPreparationFreshnessAndTimeout() {
  for (Phase phase : {Phase::DOCK_ACTIVATE, Phase::DOCK_OPEN}) {
    Harness harness;
    harness.reach(phase);
    const auto handoff = harness.step(0.01);
    harness.input.dock_fresh = false;
    harness.input.pose_fresh = false;
    harness.input.camera_x_m = harness.input.camera_y_m = harness.input.camera_z_m =
        std::numeric_limits<double>::quiet_NaN();
    harness.input.dock_complete = true;  // A stale occupied flag is not a success.
    const double settle = phase == Phase::DOCK_ACTIVATE
        ? harness.config.dock_activate_settle_sec : harness.config.dock_open_settle_sec;
    auto output = harness.step(settle + 0.001);
    checkDockHold(output, phase, handoff);
    harness.input.dock_active = harness.input.dock_relay_active = true;
    harness.input.dock_latch_open = phase == Phase::DOCK_OPEN;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    checkDockHold(output, phase, handoff);
    harness.input.dock_fresh = true;
    harness.input.dock_complete = false;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    CHECK(output.phase == (phase == Phase::DOCK_ACTIVATE ? Phase::DOCK_OPEN : Phase::INSERT));
    CHECK(output.dock_command == (phase == Phase::DOCK_ACTIVATE
        ? DockCommand::OPEN_LATCH : DockCommand::NONE));

    Harness timeout;
    timeout.reach(phase);
    timeout.input.dock_fresh = false;
    output = timeout.step(timeout.config.dock_prepare_timeout_sec - 0.1);
    CHECK(output.phase == phase);  // Each stage receives its own full timeout.
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::NONE);
    output = timeout.step(0.2);
    CHECK(output.phase == Phase::DOCK_RECOVER);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(output.dock_retry_attempt == 0);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    output = timeout.step(timeout.config.dock_prepare_timeout_sec + 0.001);
    CHECK(output.phase == Phase::FAILED);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(output.dock_retry_attempt == 0);  // Never reset an unknown/stale dock.
  }
}

void testDockPreparationContactPriority() {
  for (Phase phase : {Phase::DOCK_ACTIVATE, Phase::DOCK_OPEN,
                     Phase::DOCK_RECOVER, Phase::DOCK_DEACTIVATE}) {
    for (int contactKind = 0; contactKind < 3; ++contactKind) {
      Harness harness;
      harness.reach(phase);
      harness.input.dock_fresh = true;
      harness.input.pose_fresh = false;
      harness.input.dock_detected = contactKind == 0;
      harness.input.dock_closing = contactKind == 1;
      harness.input.dock_complete = contactKind == 2;
      // Contact outranks both a preparation deadline and a pending command.
      const auto output = harness.step(harness.config.dock_prepare_timeout_sec);
      CHECK(output.phase == Phase::LATCH_DEACTIVATE);
      CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
      CHECK(output.dock_command == DockCommand::IDLE);
      CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    }
  }

  // Contact at the distance trigger switches directly to latch-only recovery.
  Harness handoffContact;
  handoffContact.approach();
  handoffContact.input.camera_z_m = handoffContact.config.insertion_distance_m;
  handoffContact.input.dock_detected = true;
  const auto output = handoffContact.step();
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);
  CHECK(output.dock_command == DockCommand::IDLE);
}

void testDockPreparationTimingValidation() {
  for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    Controller::Config config;
    config.dock_activate_settle_sec = invalid;
    Harness activation(config);
    CHECK(activation.step().phase == Phase::FAILED);
    config = Controller::Config{};
    config.dock_open_settle_sec = invalid;
    Harness opening(config);
    CHECK(opening.step().phase == Phase::FAILED);
    config = Controller::Config{};
    config.dock_deactivate_settle_sec = invalid;
    Harness deactivating(config);
    CHECK(deactivating.step().phase == Phase::FAILED);
  }
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    Controller::Config config;
    config.dock_prepare_timeout_sec = invalid;
    Harness harness(config);
    CHECK(harness.step().phase == Phase::FAILED);
  }
  for (int stage = 0; stage < 3; ++stage) {
    Controller::Config config;
    if (stage == 0) config.dock_activate_settle_sec = config.dock_prepare_timeout_sec;
    else if (stage == 1) config.dock_open_settle_sec = config.dock_prepare_timeout_sec;
    else config.dock_deactivate_settle_sec = config.dock_prepare_timeout_sec;
    Harness harness(config);
    CHECK(harness.step().phase == Phase::FAILED);
  }
  Controller::Config noDwell;
  noDwell.dock_activate_settle_sec = noDwell.dock_open_settle_sec =
      noDwell.dock_deactivate_settle_sec = 0.0;
  Harness harness(noDwell);
  CHECK(harness.insert().phase == Phase::INSERT);
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  harness.recoverToDeactivate();
  harness.deactivateToActivate();
  CHECK(harness.finishPreparation().phase == Phase::INSERT);
}

Controller::Output waitForSpacedCommand(Harness& harness, Phase phase,
                                      DockCommand command,
                                      const Controller::Output& held,
                                      double& last_command_sec) {
  const double earliest = last_command_sec + harness.config.dock_command_interval_sec;
  for (int tick = 0; tick < 160; ++tick) {
    const auto output = harness.step(0.025);
    CHECK(output.phase == phase);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(near(output.pitch_rad, held.pitch_rad));
    CHECK(near(output.yaw_rad, held.yaw_rad));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    if (output.dock_command == DockCommand::NONE) {
      CHECK(harness.input.now_sec < earliest + 1e-9);
      continue;
    }
    CHECK(output.dock_command == command);
    CHECK(harness.input.now_sec + 1e-9 >= earliest);
    last_command_sec = harness.input.now_sec;
    // The next control tick must not retransmit the last command.
    CHECK(harness.step(0.0).dock_command == DockCommand::NONE);
    return output;
  }
  CHECK(false);
  return Controller::Output{};
}

Controller::Output acknowledgeAfterSettle(Harness& harness, double settle_sec) {
  auto output = harness.step(settle_sec + 0.001);
  CHECK(output.dock_command == DockCommand::NONE);
  ++harness.input.dock_state_sequence;
  return harness.step(0.01);
}

void testProductionDockCommandDwell() {
  Harness harness(Controller::Config{}, true);
  CHECK(near(harness.config.dock_command_interval_sec, 2.0));
  CHECK(near(harness.config.dock_deactivate_settle_sec, 2.0));
  CHECK(near(harness.config.dock_activate_settle_sec, 2.0));
  CHECK(near(harness.config.dock_open_settle_sec, 1.0));
  CHECK(near(harness.config.latch_close_settle_sec, 1.0));
  auto output = harness.prepare();
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  const double activated_sec = harness.input.now_sec;
  output = harness.activateToOpen();
  CHECK(harness.input.now_sec - activated_sec >= 2.0);
  const double opened_sec = harness.input.now_sec;
  CHECK(harness.openToInsert().phase == Phase::INSERT);
  harness.input.dock_detected = true;
  output = harness.step(0.01);
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);
  CHECK(output.dock_command == DockCommand::NONE);
  double last_command_sec = opened_sec;
  output = waitForSpacedCommand(harness, Phase::LATCH_DEACTIVATE,
                               DockCommand::IDLE, output, last_command_sec);
  const double deactivated_sec = last_command_sec;
  output = harness.latchDeactivateToActivate();
  CHECK(harness.input.now_sec - deactivated_sec >= 2.0);
  const double reactivated_sec = harness.input.now_sec;
  output = harness.latchActivateToClose();
  CHECK(harness.input.now_sec - reactivated_sec >= 2.0);
  CHECK(harness.closeToComplete().phase == Phase::COMPLETE);
}

void testContactSupersedesQueuedOpenAndSpacesLatchCommands() {
  auto config = fastFixtureConfig();
  config.dock_command_interval_sec = 2.0;
  Harness harness(config, true);
  const auto handoff = harness.prepare();
  CHECK(handoff.dock_command == DockCommand::ACTIVATE);
  double last_command_sec = harness.input.now_sec;
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  auto output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
  CHECK(output.phase == Phase::DOCK_OPEN && output.dock_command == DockCommand::NONE);
  // Contact arrives while OPEN is queued. It must cancel OPEN immediately,
  // freeze the measured joints, and delay only the replacement IDLE command.
  harness.input.dock_detected = true;
  harness.input.pitch_rad += 0.02;
  harness.input.yaw_rad += 0.01;
  const auto contact = harness.step(0.01);
  checkDockHold(contact, Phase::LATCH_DEACTIVATE, contact);
  CHECK(near(contact.pitch_rad, harness.input.pitch_rad));
  CHECK(near(contact.yaw_rad, harness.input.yaw_rad));
  harness.idleDock();
  output = waitForSpacedCommand(harness, Phase::LATCH_DEACTIVATE,
                               DockCommand::IDLE, contact, last_command_sec);
  output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact);
  output = waitForSpacedCommand(harness, Phase::LATCH_ACTIVATE,
                               DockCommand::ACTIVATE, contact, last_command_sec);
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  harness.input.dock_latch_pulse_width_us = config.latch_closed_pulse_width_us;
  output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
  output = waitForSpacedCommand(harness, Phase::WAIT_FOR_LATCH,
                               DockCommand::CLOSE_LATCH, contact, last_command_sec);
  // Failed closing uses the same paced power cycle with its own retry count.
  harness.input.dock_latch_pulse_width_us = 1500;
  output = harness.step(config.latch_timeout_sec + 0.001);
  checkDockHold(output, Phase::LATCH_RECOVER, contact);
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  checkDockHold(output, Phase::LATCH_DEACTIVATE, contact, DockCommand::IDLE);
  CHECK(output.latch_retry_attempt == 1);
  CHECK(harness.input.now_sec - last_command_sec >= config.dock_command_interval_sec);
  last_command_sec = harness.input.now_sec;
  harness.idleDock();
  output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact);
  output = waitForSpacedCommand(harness, Phase::LATCH_ACTIVATE,
                               DockCommand::ACTIVATE, contact, last_command_sec);
  CHECK(output.latch_retry_attempt == 1);
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  harness.input.dock_latch_pulse_width_us = config.latch_closed_pulse_width_us;
  output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
  waitForSpacedCommand(harness, Phase::WAIT_FOR_LATCH,
                       DockCommand::CLOSE_LATCH, contact, last_command_sec);
  // Button release cannot turn the delayed CLOSE into insertion/open.
  harness.input.dock_detected = false;
  CHECK(harness.closeToComplete().phase == Phase::COMPLETE);
}

void testQueuedDockPreparationStartsTimeoutAtEmission() {
  for (bool acknowledge_open : {false, true}) {
    auto config = fastFixtureConfig();
    config.dock_command_interval_sec = 2.0;
    config.dock_activate_settle_sec = config.dock_open_settle_sec =
        config.dock_deactivate_settle_sec = 0.1;
    config.dock_prepare_timeout_sec = 0.6;
    Harness harness(config, true);
    const auto handoff = harness.prepare();
    double last_command_sec = harness.input.now_sec;
    harness.input.dock_idle = false;
    harness.input.dock_active = harness.input.dock_relay_active = true;
    auto output = acknowledgeAfterSettle(harness, config.dock_activate_settle_sec);
    checkDockHold(output, Phase::DOCK_OPEN, handoff);
    // Longer than the operation timeout, but OPEN has not been sent yet.
    output = harness.step(1.0);
    checkDockHold(output, Phase::DOCK_OPEN, handoff);
    output = waitForSpacedCommand(harness, Phase::DOCK_OPEN,
                                 DockCommand::OPEN_LATCH, handoff, last_command_sec);
    if (acknowledge_open) {
      harness.input.dock_latch_open = true;
      output = acknowledgeAfterSettle(harness, config.dock_open_settle_sec);
      CHECK(output.phase == Phase::INSERT);
      CHECK(output.dock_command == DockCommand::NONE);
    } else {
      harness.input.dock_fresh = false;
      output = harness.step(0.59);
      checkDockHold(output, Phase::DOCK_OPEN, handoff);
      CHECK(harness.step(0.02).phase == Phase::DOCK_RECOVER);
    }
  }
}

void testDockRetryCommandsRespectSpacing() {
  auto config = fastFixtureConfig();
  config.dock_command_interval_sec = 2.0;
  Harness harness(config, true);
  const auto held = harness.prepare();
  double last_command_sec = harness.input.now_sec;
  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  CHECK(acknowledgeAfterSettle(harness, config.dock_activate_settle_sec).dock_command ==
        DockCommand::NONE);
  waitForSpacedCommand(harness, Phase::DOCK_OPEN, DockCommand::OPEN_LATCH,
                      held, last_command_sec);
  CHECK(harness.openToInsert().phase == Phase::INSERT);
  harness.input.dock_latch_open = false;
  CHECK(harness.step(0.01).phase == Phase::DOCK_RECOVER);
  harness.idleDock();
  ++harness.input.dock_state_sequence;
  auto output = harness.step(0.01);
  CHECK(output.phase == Phase::DOCK_DEACTIVATE && output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 1);
  output = waitForSpacedCommand(harness, Phase::DOCK_DEACTIVATE,
                               DockCommand::IDLE, held, last_command_sec);
  CHECK(output.dock_retry_attempt == 1);  // Waiting never spends another retry.
  output = acknowledgeAfterSettle(harness, config.dock_deactivate_settle_sec);
  CHECK(output.phase == Phase::DOCK_ACTIVATE && output.dock_command == DockCommand::NONE);
  output = waitForSpacedCommand(harness, Phase::DOCK_ACTIVATE,
                               DockCommand::ACTIVATE, held, last_command_sec);
  CHECK(output.dock_retry_attempt == 1);
}

void testDockCommandIntervalValidation() {
  for (double invalid : {-0.01, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    auto config = fastFixtureConfig();
    config.dock_command_interval_sec = invalid;
    Harness harness(config, true);
    const auto output = harness.step();
    CHECK(output.phase == Phase::FAILED && output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
  }
}

void testDockInsertionHeartbeatRecovery() {
  Harness harness;
  const auto handoff = harness.insert();
  harness.input.pose_fresh = false;
  harness.input.camera_x_m = harness.input.camera_y_m = harness.input.camera_z_m =
      std::numeric_limits<double>::quiet_NaN();
  harness.input.pitch_rad += 0.03;
  harness.input.yaw_rad += 0.02;
  harness.input.extension_m += 0.02;
  harness.input.dock_fresh = false;
  auto output = harness.step();
  checkDockHold(output, Phase::DOCK_RECOVER, handoff);
  CHECK(output.dock_retry_attempt == 0);
  output = harness.step(0.2);
  checkDockHold(output, Phase::DOCK_RECOVER, handoff);

  // Freshness alone cannot reuse the reply cached when recovery began.
  harness.input.dock_fresh = true;
  output = harness.step();
  checkDockHold(output, Phase::DOCK_RECOVER, handoff);
  ++harness.input.dock_state_sequence;
  output = harness.step();
  CHECK(output.phase == Phase::INSERT);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 0);
  CHECK(near(output.pitch_rad, handoff.pitch_rad));
  CHECK(near(output.yaw_rad, handoff.yaw_rad));
  CHECK(near(output.handoff_camera_z_m, handoff.handoff_camera_z_m));
  CHECK(output.zipper_motor_velocity_rad_s > 0.0);

  // Repeated brief transport gaps consume no power-cycle retry budget.
  for (int gap = 0; gap < 5; ++gap) {
    harness.input.dock_fresh = false;
    checkDockHold(harness.step(), Phase::DOCK_RECOVER, handoff);
    harness.input.dock_fresh = true;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.dock_retry_attempt == 0);
    CHECK(output.dock_command == DockCommand::NONE);
  }
}

void testDockInsertionNotReadyRecovery() {
  // Fresh state may still be unsuitable: inactive power, relay off, closed
  // latch, or unknown occupancy each stops insertion before any reset command.
  for (int missingFlag = 0; missingFlag < 4; ++missingFlag) {
    Harness harness;
    const auto handoff = harness.insert();
    harness.input.dock_active = missingFlag != 0;
    harness.input.dock_relay_active = missingFlag != 1;
    harness.input.dock_latch_open = missingFlag != 2;
    harness.input.dock_unoccupied = missingFlag != 3;
    ++harness.input.dock_state_sequence;
    auto output = harness.step();
    checkDockHold(output, Phase::DOCK_RECOVER, handoff);
    CHECK(output.dock_retry_attempt == 0);
    // Even when the cached state was unoccupied, require a later receipt.
    output = harness.step();
    checkDockHold(output, Phase::DOCK_RECOVER, handoff);
    output = harness.recoverToDeactivate();
    checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff, DockCommand::IDLE);
    CHECK(output.dock_retry_attempt == 1);
    output = harness.deactivateToActivate();
    checkDockHold(output, Phase::DOCK_ACTIVATE, handoff, DockCommand::ACTIVATE);
    output = harness.activateToOpen();
    checkDockHold(output, Phase::DOCK_OPEN, handoff, DockCommand::OPEN_LATCH);
    output = harness.openToInsert();
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.dock_retry_attempt == 1);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(near(output.pitch_rad, handoff.pitch_rad));
    CHECK(near(output.yaw_rad, handoff.yaw_rad));
  }
}

void testDockDeactivationAcknowledgement() {
  Harness harness;
  const auto handoff = harness.insert();
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  auto output = harness.recoverToDeactivate();
  checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff, DockCommand::IDLE);
  output = harness.step(0.1);
  checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff);
  ++harness.input.dock_state_sequence;  // A pre-deadline IDLE reply is too early.
  output = harness.step(harness.config.dock_deactivate_settle_sec);
  checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff);
  output = harness.step();
  checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff);
  ++harness.input.dock_state_sequence;
  output = harness.step();
  checkDockHold(output, Phase::DOCK_ACTIVATE, handoff, DockCommand::ACTIVATE);
  CHECK(output.dock_retry_attempt == 1);

  for (int invalidFlag = 0; invalidFlag < 3; ++invalidFlag) {
    Harness invalid;
    invalid.reach(Phase::DOCK_DEACTIVATE);
    const auto original = invalid.step(invalid.config.dock_deactivate_settle_sec + 0.001);
    invalid.input.dock_idle = invalidFlag != 0;
    invalid.input.dock_relay_active = invalidFlag == 1;
    invalid.input.dock_unoccupied = invalidFlag != 2;
    ++invalid.input.dock_state_sequence;
    output = invalid.step();
    checkDockHold(output, Phase::DOCK_DEACTIVATE, original);
    invalid.idleDock();
    ++invalid.input.dock_state_sequence;
    output = invalid.step();
    checkDockHold(output, Phase::DOCK_ACTIVATE, original, DockCommand::ACTIVATE);
  }
}

void testDockPreparationTimeoutRetry() {
  for (Phase phase : {Phase::DOCK_ACTIVATE, Phase::DOCK_OPEN, Phase::DOCK_DEACTIVATE}) {
    Harness harness;
    harness.reach(phase);
    const auto original = harness.step(0.01);
    const int attempts = original.dock_retry_attempt;
    auto output = harness.step(harness.config.dock_prepare_timeout_sec);
    checkDockHold(output, Phase::DOCK_RECOVER, original);
    CHECK(output.dock_retry_attempt == attempts);

    // A stage timeout must repeat preparation even if the latest port reply
    // now looks ready; direct insertion is reserved for an insertion heartbeat gap.
    harness.input.dock_idle = false;
    harness.input.dock_fresh = harness.input.dock_unoccupied = harness.input.dock_active =
        harness.input.dock_relay_active = harness.input.dock_latch_open = true;
    ++harness.input.dock_state_sequence;
    output = harness.step();
    checkDockHold(output, Phase::DOCK_DEACTIVATE, original, DockCommand::IDLE);
    CHECK(output.dock_retry_attempt == attempts + 1);
  }
}

void testDockRecoveryRequiresSafeFreshState() {
  Harness harness;
  const auto handoff = harness.insert();
  harness.input.dock_fresh = false;
  checkDockHold(harness.step(), Phase::DOCK_RECOVER, handoff);
  ++harness.input.dock_state_sequence;
  auto output = harness.step();
  checkDockHold(output, Phase::DOCK_RECOVER, handoff);
  CHECK(output.dock_retry_attempt == 0);
  harness.input.dock_fresh = true;
  harness.input.dock_unoccupied = false;
  ++harness.input.dock_state_sequence;
  output = harness.step();
  checkDockHold(output, Phase::DOCK_RECOVER, handoff);
  CHECK(output.dock_retry_attempt == 0);
  output = harness.step(harness.config.dock_prepare_timeout_sec);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 0);
  CHECK(output.status.find("cannot confirm an unoccupied port") != std::string::npos);

  // A ready reply arriving on the first deadline update is too late, whether
  // it would otherwise resume insertion or authorize an IDLE reset command.
  for (bool readyForInsertion : {true, false}) {
    Harness late;
    late.insert();
    late.input.dock_fresh = false;
    late.input.now_sec = 10.0;  // Exact origin keeps the five-second boundary exact.
    CHECK(late.controller.update(late.input).phase == Phase::DOCK_RECOVER);
    if (readyForInsertion) late.input.dock_fresh = true;
    else late.idleDock();
    ++late.input.dock_state_sequence;
    late.input.now_sec = 10.0 + late.config.dock_prepare_timeout_sec;
    const auto expired = late.controller.update(late.input);
    CHECK(expired.phase == Phase::FAILED);
    CHECK(expired.actuation == Actuation::STOP);
    CHECK(expired.dock_command == DockCommand::NONE);
    CHECK(expired.dock_retry_attempt == 0);
    CHECK(near(expired.zipper_motor_velocity_rad_s, 0.0));
  }
}

void testDockRetryBudgetAndHeldPose() {
  Harness harness;
  CHECK(harness.config.dock_retry_attempts == 4);
  const auto handoff = harness.insert();
  harness.input.pose_fresh = false;
  harness.input.camera_x_m = harness.input.camera_y_m = harness.input.camera_z_m =
      std::numeric_limits<double>::quiet_NaN();
  for (int attempt = 1; attempt <= 4; ++attempt) {
    harness.input.pitch_rad += 0.01;
    harness.input.yaw_rad += 0.01;
    harness.input.extension_m += 0.02;
    harness.input.dock_latch_open = false;
    ++harness.input.dock_state_sequence;
    auto output = harness.step();
    checkDockHold(output, Phase::DOCK_RECOVER, handoff);
    CHECK(output.dock_retry_attempt == attempt - 1);
    output = harness.recoverToDeactivate();
    checkDockHold(output, Phase::DOCK_DEACTIVATE, handoff, DockCommand::IDLE);
    CHECK(output.dock_retry_attempt == attempt);
    output = harness.deactivateToActivate();
    checkDockHold(output, Phase::DOCK_ACTIVATE, handoff, DockCommand::ACTIVATE);
    CHECK(output.dock_retry_attempt == attempt);
    output = harness.activateToOpen();
    checkDockHold(output, Phase::DOCK_OPEN, handoff, DockCommand::OPEN_LATCH);
    CHECK(output.dock_retry_attempt == attempt);
    output = harness.openToInsert();
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.dock_retry_attempt == attempt);  // Success does not reset the budget.
    CHECK(near(output.pitch_rad, handoff.pitch_rad));
    CHECK(near(output.yaw_rad, handoff.yaw_rad));
  }

  // Exhausting the reset budget still permits a heartbeat-only resumption.
  harness.input.dock_fresh = false;
  checkDockHold(harness.step(), Phase::DOCK_RECOVER, handoff);
  harness.input.dock_fresh = true;
  ++harness.input.dock_state_sequence;
  auto output = harness.step();
  CHECK(output.phase == Phase::INSERT && output.dock_retry_attempt == 4);
  CHECK(output.dock_command == DockCommand::NONE);

  // The fifth requested reset is rejected without emitting a fifth IDLE.
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  harness.idleDock();
  ++harness.input.dock_state_sequence;
  output = harness.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.dock_retry_attempt == 4);
  CHECK(output.status.find("exhausted 4") != std::string::npos);
  CHECK(near(output.pitch_rad, handoff.pitch_rad));
  CHECK(near(output.yaw_rad, handoff.yaw_rad));

  harness.controller.reset();
  output = harness.step();
  CHECK(output.phase == Phase::IDLE && output.dock_retry_attempt == 0);
}

void testLatchRecoveryPreservesContactContext() {
  Harness harness;
  harness.insert();
  harness.input.dock_detected = true;
  const auto contact = harness.step();
  CHECK(contact.phase == Phase::LATCH_DEACTIVATE);
  CHECK(contact.dock_command == DockCommand::IDLE);
  harness.latchDeactivateToActivate();
  harness.latchActivateToClose();
  harness.input.pitch_rad += 0.01;
  harness.input.yaw_rad += 0.01;
  harness.input.dock_fresh = false;
  auto output = harness.step(harness.config.latch_timeout_sec + 0.001);
  checkDockHold(output, Phase::LATCH_RECOVER, contact);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.latch_retry_attempt == 0);

  // IDLE and button release may remove the station's contact flags. The core
  // still closes the latch and never returns to insertion or opens it again.
  harness.input.dock_detected = false;
  harness.idleDock();
  ++harness.input.dock_state_sequence;
  output = harness.step();
  checkDockHold(output, Phase::LATCH_DEACTIVATE, contact, DockCommand::IDLE);
  CHECK(output.latch_retry_attempt == 1);
  CHECK(output.dock_retry_attempt == 0);
  output = harness.latchDeactivateToActivate();
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact, DockCommand::ACTIVATE);
  output = harness.latchActivateToClose();
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact, DockCommand::CLOSE_LATCH);
  CHECK(!harness.input.dock_complete && !harness.input.dock_detected);
  output = harness.closeToComplete();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.latch_retry_attempt == 1);
}

void testLatchCycleRejectsRetainedCompleteAndCachedState() {
  Harness harness;
  harness.insert();
  harness.input.dock_complete = true;
  harness.input.dock_detected = false;
  harness.input.dock_latch_open = false;
  harness.input.dock_latch_pulse_width_us = harness.config.latch_closed_pulse_width_us;
  const auto contact = harness.step();
  checkDockHold(contact, Phase::LATCH_DEACTIVATE, contact, DockCommand::IDLE);
  CHECK(contact.latch_retry_attempt == 0);

  // A pre-existing software DOCKED/closed report must not immediately complete
  // or repeatedly issue IDLE while waiting for the requested power-off state.
  auto output = harness.step(harness.config.dock_deactivate_settle_sec + 0.001);
  checkDockHold(output, Phase::LATCH_DEACTIVATE, contact);
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  checkDockHold(output, Phase::LATCH_DEACTIVATE, contact);

  harness.input.dock_idle = true;
  harness.input.dock_active = harness.input.dock_relay_active = false;
  harness.input.dock_complete = false;
  harness.input.dock_latch_pulse_width_us = 0;
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact, DockCommand::ACTIVATE);

  harness.input.dock_idle = false;
  harness.input.dock_active = harness.input.dock_relay_active = true;
  harness.input.dock_latch_pulse_width_us = harness.config.latch_closed_pulse_width_us;
  harness.input.dock_complete = true;  // Old DOCKED cannot replace explicit CLOSE.
  output = harness.step(harness.config.dock_activate_settle_sec + 0.001);
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact);
  output = harness.step(0.01);
  checkDockHold(output, Phase::LATCH_ACTIVATE, contact);  // No later state yet.
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact, DockCommand::CLOSE_LATCH);

  // Even matching DOCKED and PWM flags need the close wait and a later receipt.
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
  output = harness.step(harness.config.latch_close_settle_sec + 0.001);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
  output = harness.step(0.01);
  checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
  ++harness.input.dock_state_sequence;
  output = harness.step(0.01);
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
}

void testLatchClosingRequiresFreshPowerAndPulse() {
  for (int missing = 0; missing < 5; ++missing) {
    Harness harness;
    harness.reach(Phase::WAIT_FOR_LATCH);
    const auto contact = harness.step(0.01);
    // Button release and UNDOCKED after a reset do not discard contact context.
    harness.input.dock_detected = harness.input.dock_complete = harness.input.dock_closing = false;
    harness.input.dock_unoccupied = true;
    harness.input.dock_fresh = missing != 0;
    harness.input.dock_active = missing != 1;
    harness.input.dock_relay_active = missing != 2;
    harness.input.dock_latch_open = missing == 3;
    harness.input.dock_latch_pulse_width_us = missing == 4 ? 1500 :
        harness.config.latch_closed_pulse_width_us;
    auto output = harness.step(harness.config.latch_close_settle_sec + 0.001);
    checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);
    ++harness.input.dock_state_sequence;
    output = harness.step(0.01);
    checkDockHold(output, Phase::WAIT_FOR_LATCH, contact);

    harness.input.dock_fresh = harness.input.dock_active = harness.input.dock_relay_active = true;
    harness.input.dock_latch_open = false;
    harness.input.dock_latch_pulse_width_us = harness.config.latch_closed_pulse_width_us;
    ++harness.input.dock_state_sequence;
    output = harness.step(0.01);
    CHECK(output.phase == Phase::COMPLETE);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
  }
}

void testLatchRecoveryTimeoutAndNewReceipt() {
  for (Phase phase : {Phase::LATCH_DEACTIVATE, Phase::LATCH_ACTIVATE, Phase::WAIT_FOR_LATCH}) {
    Harness harness;
    harness.reach(phase);
    harness.input.dock_fresh = false;
    const double timeout = phase == Phase::WAIT_FOR_LATCH
        ? harness.config.latch_timeout_sec : harness.config.dock_prepare_timeout_sec;
    auto output = harness.step(timeout + 0.001);
    CHECK(output.phase == Phase::LATCH_RECOVER);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(output.latch_retry_attempt == 0);

    // Cached flags, even occupied ones, cannot initiate another power cycle.
    harness.input.dock_fresh = harness.input.dock_complete = true;
    output = harness.step(0.01);
    CHECK(output.phase == Phase::LATCH_RECOVER);
    CHECK(output.dock_command == DockCommand::NONE);
    ++harness.input.dock_state_sequence;
    output = harness.step(0.01);
    CHECK(output.phase == Phase::LATCH_DEACTIVATE);
    CHECK(output.dock_command == DockCommand::IDLE);
    CHECK(output.latch_retry_attempt == 1);
  }

  for (bool lateFreshState : {false, true}) {
    Harness silent;
    silent.reach(Phase::LATCH_RECOVER);
    silent.input.dock_fresh = lateFreshState;
    if (lateFreshState) ++silent.input.dock_state_sequence;
    const auto output = silent.step(silent.config.dock_prepare_timeout_sec + 0.001);
    CHECK(output.phase == Phase::FAILED);  // A late receipt cannot bypass the deadline.
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(output.latch_retry_attempt == 0);
  }
}

void testLatchRetryBudgetIndependentOfApproach() {
  Harness harness;
  harness.insert();
  for (int attempt = 1; attempt <= harness.config.dock_retry_attempts; ++attempt) {
    harness.input.dock_latch_open = false;
    ++harness.input.dock_state_sequence;
    CHECK(harness.step().phase == Phase::DOCK_RECOVER);
    CHECK(harness.recoverToDeactivate().dock_retry_attempt == attempt);
    harness.deactivateToActivate();
    harness.finishPreparation();
  }

  harness.input.dock_detected = true;
  const auto contact = harness.step();
  CHECK(contact.phase == Phase::LATCH_DEACTIVATE);
  CHECK(contact.dock_retry_attempt == 4 && contact.latch_retry_attempt == 0);
  harness.input.pose_fresh = false;
  harness.input.dock_detected = false;
  harness.input.pitch_rad += 0.01;
  harness.input.yaw_rad += 0.01;
  for (int attempt = 1; attempt <= harness.config.dock_retry_attempts; ++attempt) {
    harness.latchDeactivateToActivate();
    const auto closing = harness.latchActivateToClose();
    checkDockHold(closing, Phase::WAIT_FOR_LATCH, contact, DockCommand::CLOSE_LATCH);
    harness.input.dock_latch_pulse_width_us = 1500;
    auto output = harness.step(harness.config.latch_timeout_sec + 0.001);
    checkDockHold(output, Phase::LATCH_RECOVER, contact);
    ++harness.input.dock_state_sequence;
    output = harness.step(0.01);
    checkDockHold(output, Phase::LATCH_DEACTIVATE, contact, DockCommand::IDLE);
    CHECK(output.latch_retry_attempt == attempt);
    CHECK(output.dock_retry_attempt == 4);
  }

  harness.latchDeactivateToActivate();
  harness.latchActivateToClose();
  harness.input.dock_latch_pulse_width_us = 1500;
  CHECK(harness.step(harness.config.latch_timeout_sec + 0.001).phase == Phase::LATCH_RECOVER);
  ++harness.input.dock_state_sequence;
  auto output = harness.step(0.01);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.latch_retry_attempt == 4 && output.dock_retry_attempt == 4);
  CHECK(output.status.find("Latch closing exhausted 4") != std::string::npos);
  CHECK(near(output.pitch_rad, contact.pitch_rad));
  CHECK(near(output.yaw_rad, contact.yaw_rad));
  harness.controller.reset();
  output = harness.step();
  CHECK(output.phase == Phase::IDLE);
  CHECK(output.latch_retry_attempt == 0 && output.dock_retry_attempt == 0);
}

void testLatchConfiguration() {
  for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    Controller::Config config;
    config.latch_close_settle_sec = invalid;
    Harness harness(config);
    CHECK(harness.step().phase == Phase::FAILED);
  }
  for (int invalid : {0, -1000}) {
    Controller::Config config;
    config.latch_closed_pulse_width_us = invalid;
    Harness harness(config);
    CHECK(harness.step().phase == Phase::FAILED);
  }
  Controller::Config tooLong;
  tooLong.latch_close_settle_sec = tooLong.latch_timeout_sec;
  Harness invalidTimeout(tooLong);
  CHECK(invalidTimeout.step().phase == Phase::FAILED);

  Controller::Config disabled;
  disabled.dock_retry_attempts = 0;
  Harness noRetries(disabled);
  noRetries.reach(Phase::LATCH_RECOVER);
  noRetries.input.dock_fresh = true;
  ++noRetries.input.dock_state_sequence;
  const auto output = noRetries.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.dock_command == DockCommand::NONE);
  CHECK(output.latch_retry_attempt == 0);
}

void testDockRetryConfiguration() {
  Controller::Config invalid;
  invalid.dock_retry_attempts = -1;
  Harness badConfig(invalid);
  CHECK(badConfig.step().phase == Phase::FAILED);

  Controller::Config disabled;
  disabled.dock_retry_attempts = 0;
  Harness harness(disabled);
  harness.insert();
  harness.input.dock_fresh = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  harness.input.dock_fresh = true;
  ++harness.input.dock_state_sequence;
  auto output = harness.step();
  CHECK(output.phase == Phase::INSERT && output.dock_retry_attempt == 0);
  harness.input.dock_latch_open = false;
  CHECK(harness.step().phase == Phase::DOCK_RECOVER);
  harness.idleDock();
  ++harness.input.dock_state_sequence;
  output = harness.step();
  CHECK(output.phase == Phase::FAILED && output.dock_retry_attempt == 0);
  CHECK(output.dock_command == DockCommand::NONE);
}

void testInsertionUntilContact() {
  Harness harness;
  auto output = harness.insert();
  const double heldPitch = output.pitch_rad;
  const double heldYaw = output.yaw_rad;
  harness.input.pose_fresh = false;
  harness.input.camera_z_m = std::numeric_limits<double>::quiet_NaN();
  for (double extension : {0.9, 2.0, 100.0}) {
    harness.input.extension_m = extension;
    harness.input.pitch_rad += 0.01;
    harness.input.yaw_rad += 0.01;
    output = harness.step(500.0);
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);
    CHECK(near(output.extension_m, extension));
    CHECK(near(output.pitch_rad, heldPitch));
    CHECK(near(output.yaw_rad, heldYaw));
    CHECK(output.zipper_motor_velocity_rad_s > 0.0);
  }
  harness.input.dock_detected = true;
  output = harness.step();
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::IDLE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  const auto contactPose = output;
  harness.input.extension_m += 0.001;
  output = harness.step();
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.pitch_rad, contactPose.pitch_rad));
  CHECK(near(output.yaw_rad, contactPose.yaw_rad));
  harness.input.dock_complete = true;
  output = harness.step();
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);  // Retained DOCKED cannot skip closing.
  CHECK(output.dock_command == DockCommand::NONE);
  harness.latchDeactivateToActivate();
  harness.latchActivateToClose();
  output = harness.closeToComplete();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(harness.step(100.0).phase == Phase::COMPLETE);
}

void testApproachBurstAndNewCapture() {
  Harness harness;
  auto output = harness.approach();
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.5));
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(!harness.controller.needsApproachUpdate(harness.input));
  CHECK(harness.controller.allowsYawAlignment(harness.input));

  // The fast adapter hook recognizes the deadline without waiting for another
  // normal setpoint period. Even a centered pose cannot skip the burst stop.
  const double elapsed = harness.config.approach_burst_duration_sec + 0.001;
  harness.input.now_sec += elapsed;
  harness.input.camera_now_ns += static_cast<std::int64_t>(elapsed * 1e9);
  harness.input.pose_stamp_ns = harness.input.camera_now_ns;
  CHECK(harness.controller.needsApproachUpdate(harness.input));
  output = harness.controller.update(harness.input);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(!harness.controller.allowsYawAlignment(harness.input));
  const auto stopStamp = harness.input.camera_now_ns;
  const auto stoppedPose = output;

  // Repeated publications of the last transform are not a new camera capture.
  // Keep pitch/yaw held even if measured joints drift during this stopped wait.
  harness.input.pitch_rad += 0.003;
  harness.input.yaw_rad += 0.003;
  for (int index = 0; index < 3; ++index) {
    output = harness.step(0.1, false);
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(output.actuation == Actuation::CENTER_PITCH);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    CHECK(near(output.pitch_rad, stoppedPose.pitch_rad));
    CHECK(near(output.yaw_rad, stoppedPose.yaw_rad));
    CHECK(!harness.controller.allowsYawAlignment(harness.input));
  }
  harness.input.pose_stamp_ns = stopStamp;
  CHECK(harness.step(0.1, false).phase == Phase::PITCH_CENTER);
  CHECK(!harness.controller.allowsYawAlignment(harness.input));

  // Tracking loss must not discard the post-stop capture requirement.
  harness.input.pose_fresh = false;
  CHECK(harness.step(0.1, false).phase == Phase::WAIT_FOR_TAG);
  CHECK(!harness.controller.allowsYawAlignment(harness.input));
  harness.input.pose_fresh = true;
  output = harness.step(0.1, false);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(!harness.controller.allowsYawAlignment(harness.input));

  // A post-stop capture permits a pitch check; a 2.5 degree error still needs
  // correction before another burst because the centering target is 2 degrees.
  harness.input.pose_stamp_ns = stopStamp + 1;
  harness.input.camera_y_m = harness.config.goal_camera_y_m +
      harness.input.camera_z_m * std::tan(2.5 * kDeg);
  CHECK(harness.controller.allowsYawAlignment(harness.input));
  output = harness.step(0.1, false);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(output.pitch_rad < harness.input.pitch_rad);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(harness.controller.allowsYawAlignment(harness.input));
  harness.input.camera_y_m = harness.config.goal_camera_y_m;
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.pitch_rad, harness.input.pitch_rad));
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.5));
  CHECK(harness.controller.allowsYawAlignment(harness.input));

  // A valid final handoff and fresh physical contact both outrank the deadline.
  Harness finalHandoff;
  finalHandoff.approach();
  finalHandoff.input.camera_z_m = finalHandoff.config.insertion_distance_m;
  output = finalHandoff.step(elapsed);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(!finalHandoff.controller.allowsYawAlignment(finalHandoff.input));

  // A fresh near pose outranks the post-burst capture gate, even when its
  // timestamp is the same as the pose that was held at the stop.
  Harness waitingNear;
  waitingNear.approach();
  output = waitingNear.step(elapsed);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(!waitingNear.controller.allowsYawAlignment(waitingNear.input));
  waitingNear.input.camera_z_m = waitingNear.config.insertion_distance_m;
  waitingNear.input.camera_x_m = 0.04;
  waitingNear.input.camera_y_m = waitingNear.config.goal_camera_y_m +
      waitingNear.input.camera_z_m * std::tan(3.0 * kDeg);
  CHECK(waitingNear.controller.needsInsertionUpdate(waitingNear.input));
  output = waitingNear.step(0.1, false);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);
  CHECK(!waitingNear.controller.allowsYawAlignment(waitingNear.input));

  Harness staleNear;
  staleNear.approach();
  CHECK(staleNear.step(elapsed).phase == Phase::PITCH_CENTER);
  staleNear.input.camera_z_m = staleNear.config.insertion_distance_m;
  staleNear.input.pose_fresh = false;
  CHECK(!staleNear.controller.needsInsertionUpdate(staleNear.input));
  output = staleNear.step(0.1, false);
  CHECK(output.phase == Phase::WAIT_FOR_TAG);
  CHECK(output.actuation == Actuation::STOP);
  staleNear.input.pose_fresh = true;
  output = staleNear.step(0.1, false);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::ACTIVATE);

  Harness contact;
  contact.approach();
  contact.input.dock_detected = true;
  output = contact.step(elapsed);
  CHECK(output.phase == Phase::LATCH_DEACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == DockCommand::IDLE);

  Harness bounds;
  bounds.approach();
  bounds.step(elapsed);
  bounds.input.extension_m = bounds.config.extension_max_m;
  output = bounds.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
}

void testTrackingPauseAndResume() {
  Harness harness;
  auto output = harness.lose();
  CHECK(output.phase == Phase::WAIT_FOR_TAG);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.pitch_rad, harness.input.pitch_rad));
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));

  // A long gap causes no retract, pitch sweep or elapsed-time failure.
  for (int index = 0; index < 3; ++index) {
    output = harness.step(500.0);
    CHECK(output.phase == Phase::WAIT_FOR_TAG);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(near(output.extension_m, harness.input.extension_m));
  }
  harness.input.extension_m += 0.001;
  harness.input.pitch_rad += 0.001;
  harness.input.pose_fresh = true;
  harness.input.camera_x_m = 0.04;
  harness.input.camera_y_m = harness.config.goal_camera_y_m - 0.04;
  output = harness.step();
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(output.pitch_rad > harness.input.pitch_rad);
  CHECK(output.yaw_rad > harness.input.yaw_rad);

  harness.input.camera_y_m = harness.config.goal_camera_y_m;
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.pitch_rad, harness.input.pitch_rad));

  // Close-range tracking loss also pauses; it must not silently enter insertion.
  harness.input.camera_z_m = harness.config.insertion_distance_m +
      0.6 * harness.config.depth_deadband_m;
  harness.input.pose_fresh = false;
  CHECK(!harness.controller.needsInsertionUpdate(harness.input));
  output = harness.step();
  CHECK(output.phase == Phase::WAIT_FOR_TAG);
  CHECK(output.actuation == Actuation::STOP);
}

void testContactPriority() {
  const Phase phases[] = {Phase::SEARCH, Phase::PITCH_CENTER, Phase::APPROACH, Phase::WAIT_FOR_TAG,
    Phase::DOCK_ACTIVATE, Phase::DOCK_OPEN, Phase::DOCK_RECOVER, Phase::DOCK_DEACTIVATE,
    Phase::INSERT};
  for (Phase phase : phases) {
    Harness harness;
    harness.reach(phase);
    CHECK(harness.controller.phase() == phase);
    harness.input.dock_detected = true;
    harness.input.dock_fresh = true;
    harness.input.pose_fresh = false;
    auto output = harness.step();
    CHECK(output.phase == Phase::LATCH_DEACTIVATE);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::IDLE);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    harness.input.dock_complete = true;
    output = harness.step();
    CHECK(output.phase == Phase::LATCH_DEACTIVATE);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::NONE);
    harness.latchDeactivateToActivate();
    harness.latchActivateToClose();
    CHECK(harness.closeToComplete().phase == Phase::COMPLETE);
  }

  Harness closing;
  closing.input.dock_closing = true;
  CHECK(closing.step().phase == Phase::LATCH_DEACTIVATE);
  Harness simultaneous;
  simultaneous.insert();
  simultaneous.input.dock_detected = true;
  simultaneous.input.dock_complete = true;
  CHECK(simultaneous.step().phase == Phase::LATCH_DEACTIVATE);

  for (Phase phase : {Phase::LATCH_DEACTIVATE, Phase::LATCH_ACTIVATE,
                     Phase::WAIT_FOR_LATCH, Phase::LATCH_RECOVER}) {
    Harness latched;
    latched.reach(phase);
    latched.input.dock_fresh = latched.input.dock_detected = latched.input.dock_complete = true;
    const auto output = latched.step(0.01);
    CHECK(output.phase == phase);  // Repeated contact must not restart a phase or skip its gate.
    CHECK(output.dock_command == DockCommand::NONE);
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  }
}

void testDockFreshnessAndLatchTimeout() {
  for (Phase phase : {Phase::INSERT, Phase::WAIT_FOR_LATCH}) {
    Harness harness;
    harness.reach(phase);
    harness.input.dock_fresh = false;
    harness.input.dock_complete = true;  // Stale complete must not create success.
    const auto output = harness.step();
    CHECK(output.phase == (phase == Phase::INSERT ? Phase::DOCK_RECOVER : Phase::WAIT_FOR_LATCH));
    CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
    CHECK(output.dock_command == DockCommand::NONE);
  }

  Harness timeout;
  timeout.reach(Phase::WAIT_FOR_LATCH);
  CHECK(timeout.step(timeout.config.latch_timeout_sec + 0.001).phase == Phase::LATCH_RECOVER);
  timeout.input.dock_fresh = false;
  CHECK(timeout.step(timeout.config.dock_prepare_timeout_sec + 0.001).phase == Phase::FAILED);

  for (int occupied = 0; occupied < 3; ++occupied) {
    Harness harness;
    harness.input.dock_fresh = false;
    harness.input.dock_detected = occupied == 0;
    harness.input.dock_closing = occupied == 1;
    harness.input.dock_complete = occupied == 2;
    harness.controller.start(harness.input);
    const auto output = harness.step();
    CHECK(output.phase == Phase::FAILED);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(output.status.find(occupied == 0 ? "dock_detected=true" :
        occupied == 1 ? "docking_state=DOCKING" : "docking_state=DOCKED") != std::string::npos);
  }
}

void testValidationResetAndBurstWait() {
  // A corrupted home count must not be treated as a valid starting pose by
  // wrapping a full turn away. Report enough detail to identify the bad axis.
  Harness badHome;
  badHome.input.pitch_rad = 370.898 * kDeg;
  badHome.input.extension_m = 0.0154434;
  badHome.controller.start(badHome.input);
  auto rejected = badHome.step();
  CHECK(rejected.phase == Phase::FAILED);
  CHECK(rejected.actuation == Actuation::STOP);
  CHECK(rejected.status.find("pitch=370.898") != std::string::npos);
  CHECK(rejected.status.find("70.000000") != std::string::npos);
  CHECK(rejected.status.find("extension=0.015443") != std::string::npos);
  badHome.input.pitch_rad = 12.3047 * kDeg;
  badHome.controller.start(badHome.input);
  CHECK(badHome.controller.phase() == Phase::SEARCH);
  CHECK(badHome.step().actuation == Actuation::TRACK_POSE);

  Controller::Config invalid;
  invalid.yaw_gain = std::numeric_limits<double>::quiet_NaN();
  Harness badConfig(invalid);
  CHECK(badConfig.step().phase == Phase::FAILED);
  invalid = Controller::Config{};
  invalid.pitch_min_rad = invalid.pitch_max_rad;
  Harness badBounds(invalid);
  CHECK(badBounds.step().phase == Phase::FAILED);
  invalid = Controller::Config{};
  invalid.approach_burst_duration_sec = 0.0;
  Harness badBurst(invalid);
  CHECK(badBurst.step().phase == Phase::FAILED);

  Harness invalidJoint;
  invalidJoint.input.yaw_rad = std::numeric_limits<double>::infinity();
  CHECK(invalidJoint.step().phase == Phase::FAILED);
  Harness invalidPose;
  invalidPose.input.pose_fresh = true;
  invalidPose.input.camera_x_m = std::numeric_limits<double>::quiet_NaN();
  CHECK(invalidPose.step().phase == Phase::FAILED);
  Harness badTime;
  badTime.input.now_sec = -1.0;
  CHECK(badTime.controller.update(badTime.input).phase == Phase::FAILED);

  Harness longApproach;
  longApproach.approach();
  CHECK(longApproach.step(500.0).phase == Phase::PITCH_CENTER);
  CHECK(longApproach.step().phase == Phase::APPROACH);

  Harness reset;
  reset.insert();
  reset.controller.reset();
  CHECK(reset.controller.phase() == Phase::IDLE);
  CHECK(reset.step().actuation == Actuation::STOP);
  reset.input.pose_fresh = false;
  reset.controller.start(reset.input);
  CHECK(reset.controller.phase() == Phase::SEARCH);
  CHECK(reset.step().actuation == Actuation::TRACK_POSE);
}

void testYawAlignmentPhaseGuard() {
  Harness search;
  search.input.pose_fresh = true;
  CHECK(search.controller.phase() == Phase::SEARCH);
  CHECK(!search.controller.allowsYawAlignment(search.input));

  Harness centering;
  centering.reach(Phase::PITCH_CENTER);
  CHECK(centering.controller.allowsYawAlignment(centering.input));
  centering.input.pose_fresh = false;
  CHECK(!centering.controller.allowsYawAlignment(centering.input));

  Harness approach;
  approach.approach();
  CHECK(approach.controller.allowsYawAlignment(approach.input));
  approach.input.pose_fresh = false;
  CHECK(!approach.controller.allowsYawAlignment(approach.input));

  Harness lost;
  lost.lose();
  lost.input.pose_fresh = true;
  CHECK(!lost.controller.allowsYawAlignment(lost.input));

  for (Phase phase : {Phase::DOCK_ACTIVATE, Phase::DOCK_OPEN,
                     Phase::DOCK_RECOVER, Phase::DOCK_DEACTIVATE,
                     Phase::LATCH_DEACTIVATE, Phase::LATCH_ACTIVATE,
                     Phase::WAIT_FOR_LATCH, Phase::LATCH_RECOVER}) {
    Harness preparation;
    preparation.reach(phase);
    CHECK(preparation.input.pose_fresh);
    CHECK(!preparation.controller.allowsYawAlignment(preparation.input));
  }

  Harness insertion;
  insertion.insert();
  CHECK(insertion.input.pose_fresh);
  CHECK(!insertion.controller.allowsYawAlignment(insertion.input));
  insertion.input.dock_detected = true;
  CHECK(insertion.step().phase == Phase::LATCH_DEACTIVATE);
  CHECK(!insertion.controller.allowsYawAlignment(insertion.input));
  insertion.controller.reset();
  CHECK(!insertion.controller.allowsYawAlignment(insertion.input));
}

void testYawThreeCentimeterLateralDeadband() {
  CHECK(near(Controller::Config{}.lateral_deadband_m, 0.03));
  CHECK(near(YawController::Config{}.camera_lateral_deadband_m, 0.03));
  CHECK(near(YawController::Config{}.camera_lateral_resume_m, 0.05));

  // Both yaw layers accept the boundary, including errors that used to
  // provoke fine corrections. The deadband is relative to the camera goal.
  for (double error : {-0.03, -0.025, 0.0, 0.025, 0.03}) {
    Controller::Config settings;
    settings.goal_camera_x_m = 0.1;
    Harness core(settings);
    core.input.pose_fresh = true;
    core.input.camera_x_m = settings.goal_camera_x_m + error;
    core.input.camera_y_m = 0.08;
    const auto coreOutput = core.step();
    CHECK(coreOutput.phase == Phase::PITCH_CENTER);
    CHECK(near(coreOutput.yaw_rad, core.input.yaw_rad));

    YawHarness hold(YawController::Config{}, 0.4);
    hold.input.camera_x_error_m = error;
    const auto output = hold.step();
    CHECK(!output.adjusting && !output.waiting_for_capture && !output.failed);
    CHECK(near(output.motor_velocity_rad_s, 0.0));
    CHECK(near(output.yaw_rad, 0.4));
  }

  for (double sign : {-1.0, 1.0}) {
    YawHarness harness(YawController::Config{}, 0.4);
    harness.input.camera_x_error_m = sign * 0.04;
    auto output = harness.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.step_rad, sign * 3.0 * kDeg));
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));

    // Entering the accepted window cancels an in-flight coarse step even
    // when the pitch controller is waiting for its next camera capture.
    harness.input.yaw_rad += sign * 0.2 * kDeg;
    harness.input.camera_x_error_m = sign * 0.025;
    harness.input.allow_new_target = false;
    output = harness.step();
    CHECK(!output.adjusting && output.waiting_for_capture && !output.failed);
    CHECK(near(output.motor_velocity_rad_s, 0.0));
    const double heldYaw = output.yaw_rad;

    // A settled angle remains fixed across new poses and encoder noise;
    // the old queued request must not create another motor pulse.
    harness.input.allow_new_target = true;
    for (double error : {sign * 0.029, -sign * 0.025, sign * 0.03,
                         sign * 0.032, -sign * 0.049, sign * 0.05}) {
      harness.input.camera_x_error_m = error;
      harness.input.yaw_rad = heldYaw + sign * 0.167 * kDeg;
      output = harness.step(0.25);
      CHECK(!output.adjusting && !output.waiting_for_capture && !output.failed);
      CHECK(near(output.motor_velocity_rad_s, 0.0));
      CHECK(near(output.yaw_rad, heldYaw));
    }

    // After alignment, noise above 3 cm does not restart fine corrections.
    // Only drift beyond the 5 cm outer window resumes a full-speed step.
    harness.input.camera_x_error_m = sign * 0.06;
    output = harness.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.step_rad, sign * 3.0 * kDeg));
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));

    harness.input.camera_x_error_m = sign * 0.025;
    CHECK(!harness.step().adjusting);
    harness.input.camera_x_error_m = sign * 0.06;
    CHECK(harness.step(0.1, false).waiting_for_capture);
    CHECK(harness.step(0.11, false).waiting_for_capture);
    output = harness.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));

    // A new run starts coarse alignment afresh, including an initial error
    // between the stop and resume thresholds.
    harness.input.camera_x_error_m = sign * 0.025;
    CHECK(!harness.step().adjusting);
    harness.controller.start(harness.input.now_sec, harness.input.yaw_rad,
                             harness.input.feedback_sequence);
    harness.input.camera_x_error_m = sign * 0.04;
    output = harness.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
  }

  for (double resume : {0.029, -0.01,
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    YawController::Config invalid;
    invalid.camera_lateral_resume_m = resume;
    YawHarness harness(invalid);
    CHECK(harness.step().failed);
  }
}

void testYawSmallRequestsFullStep() {
  // The core can request less than the motor completion deadband. That still
  // allows one full small step, with its direction set by this capture.
  const double initialYaw = 0.4;
  for (double requestDegrees : {0.01, 0.05}) {
    for (double sign : {-1.0, 1.0}) {
      YawHarness harness(YawController::Config{}, initialYaw);
      harness.input.camera_x_error_m = sign * 0.05;
      harness.input.requested_yaw_rad = initialYaw + sign * requestDegrees * kDeg;
      auto output = harness.step();
      CHECK(output.adjusting && !output.failed && !output.step_timed_out);
      CHECK(near(output.yaw_rad, initialYaw + sign * harness.config.step_rad));
      CHECK(near(output.step_rad, sign * harness.config.step_rad));
      CHECK(near(output.step_motion_rad, 0.0));
      const double fixedTarget = output.yaw_rad;

      // New core requests and captures cannot accumulate a farther target
      // while the current step is still blocked at its starting angle.
      for (int index = 0; index < 20; ++index) {
        harness.input.requested_yaw_rad = initialYaw + sign * (index + 1) * kDeg;
        output = harness.step();
        CHECK(output.adjusting && !output.step_timed_out);
        CHECK(near(output.yaw_rad, fixedTarget));
        CHECK(near(output.step_rad, sign * harness.config.step_rad));
        CHECK(near(output.step_motion_rad, 0.0));
      }
    }
  }

  YawHarness zero;
  zero.input.requested_yaw_rad = zero.input.yaw_rad;
  auto output = zero.step();
  CHECK(output.adjusting && !output.waiting_for_capture);
  CHECK(near(output.yaw_rad, zero.input.yaw_rad + zero.config.step_rad));
  CHECK(near(output.step_rad, zero.config.step_rad));

  // An independently ramped core request must not veto the camera direction.
  for (double sign : {-1.0, 1.0}) {
    YawHarness mismatched;
    mismatched.input.requested_yaw_rad = -sign * 0.01 * kDeg;
    mismatched.input.camera_x_error_m = sign * 0.05;
    output = mismatched.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.step_rad, sign * mismatched.config.step_rad));
  }
}

void testYawCameraDirectionSurvivesLaggingCoreTarget() {
  const double initialYaw = 98.27 * kDeg;
  YawHarness harness(YawController::Config{}, initialYaw);
  harness.input.camera_x_error_m = -0.121;
  harness.input.requested_yaw_rad = initialYaw - 0.02 * kDeg;
  auto output = harness.step();
  CHECK(output.adjusting && near(output.yaw_rad, initialYaw - harness.config.step_rad));

  // Complete the fixed target. The old core setpoint is now on the positive
  // side of measured yaw, while the camera still requests a negative nudge.
  harness.input.yaw_rad = output.yaw_rad;
  output = harness.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(harness.input.requested_yaw_rad > harness.input.yaw_rad);
  output = harness.step(harness.config.observation_pause_sec + 0.01);
  CHECK(output.adjusting && !output.failed);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad - harness.config.step_rad));
  CHECK(near(output.motor_velocity_rad_s, -3.5));
}

void testYawCumulativeSmallStepsContinueBeyondThirtyDegrees() {
  for (double sign : {-1.0, 1.0}) {
    const double initialYaw = 510.4 * kDeg;
    YawHarness harness(YawController::Config{}, initialYaw);
    harness.input.camera_x_error_m = sign * 0.1;
    // Deliberately keep the core request fixed as encoder feedback progresses.
    harness.input.requested_yaw_rad = initialYaw + sign * 0.01 * kDeg;
    auto output = harness.step();
    for (int index = 0; index < 12; ++index) {
      CHECK(output.adjusting && !output.failed && !output.feedback_recovery_required);
      CHECK(near(output.yaw_rad, harness.input.yaw_rad + sign * harness.config.step_rad));
      harness.input.yaw_rad = output.yaw_rad;
      output = harness.step(0.02);
      CHECK(!output.adjusting && output.waiting_for_capture && !output.failed);
      CHECK(near(output.yaw_rad, harness.input.yaw_rad));
      output = harness.step(harness.config.observation_pause_sec + 0.01);
    }
    CHECK(sign * (harness.input.yaw_rad - initialYaw) > 30.0 * kDeg);
    CHECK(output.adjusting && !output.failed && !output.feedback_recovery_required);
    CHECK(near(output.step_rad, sign * harness.config.step_rad));
  }
}

void testYawTimeoutMotionReporting() {
  for (double sign : {-1.0, 1.0}) {
    const double initialYaw = 0.4;
    YawHarness harness(YawController::Config{}, initialYaw);
    harness.input.camera_x_error_m = sign * 0.05;
    harness.input.requested_yaw_rad = initialYaw + sign * 0.01 * kDeg;
    auto output = harness.step();
    CHECK(output.adjusting && !output.step_timed_out);

    // Partial progress stays farther than the target completion deadband.
    harness.input.yaw_rad = initialYaw + sign * 0.2 * kDeg;
    output = harness.step(0.2);
    CHECK(output.adjusting && !output.step_timed_out);
    CHECK(near(output.step_rad, sign * harness.config.step_rad));
    CHECK(near(output.step_motion_rad, sign * 0.2 * kDeg));
    output = harness.step(harness.config.adjustment_timeout_sec - 0.2 + 0.01, false);
    CHECK(!output.adjusting && output.waiting_for_capture && output.step_timed_out);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));
    CHECK(near(output.step_rad, sign * harness.config.step_rad));
    CHECK(near(output.step_motion_rad, sign * 0.2 * kDeg));

    // The event fires once; observing holds the final report for this step.
    harness.input.yaw_rad += sign * 0.1 * kDeg;
    output = harness.step(0.01, false);
    CHECK(!output.step_timed_out && output.waiting_for_capture);
    CHECK(near(output.step_motion_rad, sign * 0.2 * kDeg));
  }
}

void testYawStepsObservationAndDirection() {
  YawHarness harness;
  auto output = harness.step();
  CHECK(output.adjusting && !output.failed);
  CHECK(near(output.yaw_rad, harness.config.step_rad));
  const double fixedTarget = output.yaw_rad;
  harness.input.requested_yaw_rad = 3.0 * kDeg;
  output = harness.step(0.1);
  CHECK(output.adjusting);
  CHECK(near(output.yaw_rad, fixedTarget));

  // Cached camera data cannot reverse an active step; a new capture can.
  harness.input.camera_x_error_m = -0.05;
  output = harness.step(0.01, false);
  CHECK(output.adjusting && near(output.yaw_rad, fixedTarget));
  output = harness.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(!output.step_timed_out);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  output = harness.step(0.25);
  // The fresh negative camera direction overrides the old positive core target.
  CHECK(output.adjusting && near(output.yaw_rad, -harness.config.step_rad));

  // Repeated larger requests do not restart the two-second step deadline.
  YawHarness timeout;
  timeout.step();
  timeout.input.requested_yaw_rad = 10.0 * kDeg;
  CHECK(timeout.step(timeout.config.adjustment_timeout_sec - 0.01, false).adjusting);
  output = timeout.step(0.02, false);
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(output.step_timed_out);
  CHECK(near(output.yaw_rad, timeout.input.yaw_rad));
  CHECK(near(output.step_rad, timeout.config.step_rad));
  CHECK(near(output.step_motion_rad, 0.0));
  const auto stopStamp = timeout.input.camera_now_ns;
  output = timeout.step(0.21, false);
  CHECK(output.waiting_for_capture && !output.adjusting);
  CHECK(!output.step_timed_out);
  timeout.input.pose_stamp_ns = stopStamp;
  CHECK(timeout.step(0.01, false).waiting_for_capture);
  timeout.input.pose_stamp_ns = timeout.input.camera_now_ns;
  output = timeout.step(0.01, false);
  CHECK(output.adjusting && near(output.yaw_rad, timeout.config.step_rad));
  CHECK(!output.step_timed_out && near(output.step_motion_rad, 0.0));

  // A new post-stop capture alone cannot skip the observation pause.
  YawHarness reached;
  output = reached.step();
  reached.input.yaw_rad = output.yaw_rad - 0.1 * kDeg;
  reached.input.requested_yaw_rad = reached.input.yaw_rad + 2.0 * kDeg;
  output = reached.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(!output.step_timed_out);
  CHECK(near(output.yaw_rad, reached.input.yaw_rad));
  CHECK(reached.step(0.1).waiting_for_capture);
  CHECK(reached.step(0.11, false).adjusting);

  // Crossing the target freezes actual yaw rather than pulling backward.
  YawHarness overshoot;
  output = overshoot.step();
  const double crossedTarget = output.yaw_rad + 0.167 * kDeg;
  overshoot.input.yaw_rad = crossedTarget;
  output = overshoot.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(!output.step_timed_out);
  CHECK(near(output.yaw_rad, crossedTarget));

  // Centered camera error cancels motion and discards queued core corrections.
  YawHarness centered;
  centered.step();
  centered.input.yaw_rad = 0.1 * kDeg;
  centered.input.camera_x_error_m = 0.004;
  output = centered.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(!output.step_timed_out);
  centered.input.requested_yaw_rad = 10.0 * kDeg;
  output = centered.step(0.25);
  CHECK(!output.adjusting && !output.waiting_for_capture);
  CHECK(near(output.yaw_rad, centered.input.yaw_rad));
  const double centeredHold = output.yaw_rad;
  centered.input.yaw_rad += 0.167 * kDeg;
  for (int tick = 0; tick < 10; ++tick) {
    output = centered.step();
    CHECK(!output.adjusting && !output.waiting_for_capture && !output.failed);
    CHECK(near(output.yaw_rad, centeredHold));
    CHECK(near(output.motor_velocity_rad_s, 0.0));
  }
}

void testYawCancellationSettledHoldAndValidation() {
  YawHarness canceled;
  canceled.step();
  canceled.input.yaw_rad = 0.2 * kDeg;
  canceled.input.allow_new_target = false;
  canceled.input.allow_alignment = false;
  auto output = canceled.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(!output.step_timed_out);
  const double canceledHold = output.yaw_rad;
  canceled.input.yaw_rad = 0.4 * kDeg;
  output = canceled.step(1.0);
  CHECK(output.waiting_for_capture && near(output.yaw_rad, canceledHold));
  canceled.input.allow_alignment = true;
  canceled.input.allow_new_target = true;
  output = canceled.step();
  CHECK(output.adjusting);  // Disabled ticks did not restart the pause.
  CHECK(near(output.yaw_rad, canceled.input.yaw_rad + canceled.config.step_rad));

  YawController::Config reverseConfig;
  reverseConfig.yaw_correction_sign = -1.0;
  YawHarness reverse(reverseConfig);
  reverse.input.requested_yaw_rad = -2.0 * kDeg;
  output = reverse.step();
  CHECK(output.adjusting && near(output.yaw_rad, -reverse.config.step_rad));

  // Paused alignment keeps one settled hold through small encoder drift.
  YawHarness idle;
  idle.input.allow_alignment = false;
  idle.input.yaw_rad = 1.0 * kDeg;
  output = idle.step();
  CHECK(!output.adjusting && near(output.yaw_rad, 0.0));
  idle.input.allow_alignment = true;
  idle.input.yaw_rad = 1.0 * kDeg + idle.config.step_rad + 0.3 * kDeg;
  output = idle.step();
  CHECK(!output.adjusting && output.waiting_for_capture);
  CHECK(near(output.yaw_rad, idle.input.yaw_rad));

  YawHarness jump;
  jump.input.yaw_rad = 5.1 * kDeg;
  output = jump.step();
  CHECK(!output.failed && output.feedback_recovery_required && output.feedback_jump_detected);
  CHECK(near(output.motor_velocity_rad_s, 0.0));
  CHECK(near(output.yaw_rad, 0.0));
  YawController::Config badConfig;
  badConfig.step_rad = 0.0;
  YawHarness invalid(badConfig);
  CHECK(invalid.step().failed);
}

void testYawCaptureWaitPreservesPositionTargetAndDeadline() {
  // A pitch/extension capture wait blocks the next nudge, rather than
  // replacing an unfinished position target with each measured angle.
  const double initialYaw = 510.4 * kDeg;
  for (double sign : {-1.0, 1.0}) {
    YawHarness harness(YawController::Config{}, initialYaw);
    harness.input.camera_x_error_m = sign * 0.1;
    harness.input.requested_yaw_rad = initialYaw + sign * 2.0 * kDeg;
    auto output = harness.step();
    CHECK(output.adjusting);
    const double fixedTarget = output.yaw_rad;
    const double startTime = harness.input.now_sec;
    harness.input.allow_new_target = false;

    for (int tick = 0; tick < 4; ++tick) {
      harness.input.yaw_rad = initialYaw + sign * tick * 0.167 * kDeg;
      harness.input.requested_yaw_rad = initialYaw + sign * (tick + 10.0) * kDeg;
      output = harness.step(0.4, false);
      CHECK(output.adjusting && !output.step_timed_out && !output.failed);
      CHECK(near(output.yaw_rad, fixedTarget));
      CHECK(near(output.step_rad, sign * harness.config.step_rad));
      CHECK(near(output.step_motion_rad, harness.input.yaw_rad - initialYaw));
    }

    // Blocking new targets cannot extend the deadline of the current target.
    const double remaining = harness.config.adjustment_timeout_sec -
        (harness.input.now_sec - startTime);
    output = harness.step(remaining + 0.01, false);
    CHECK(!output.adjusting && output.step_timed_out && output.waiting_for_capture);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));
    CHECK(near(output.step_rad, sign * harness.config.step_rad));
    CHECK(near(output.step_motion_rad, sign * 0.501 * kDeg));

    // A new camera capture and an elapsed pause cannot start another target
    // while the main docking controller still requires a capture wait.
    output = harness.step(harness.config.observation_pause_sec + 0.01);
    CHECK(!output.adjusting && !output.step_timed_out && output.waiting_for_capture);
    harness.input.allow_new_target = true;
    output = harness.step(0.01, false);
    CHECK(output.adjusting && !output.waiting_for_capture);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad + sign * harness.config.step_rad));
  }
}

void testYawQuantizedFeedbackAndBlockedObservation() {
  // A 3-degree step remains distinguishable with roughly 0.167-degree motor
  // feedback increments; progress is held until within the 0.2-degree band.
  for (double sign : {-1.0, 1.0}) {
    YawHarness harness;
    harness.input.camera_x_error_m = sign * 0.05;
    harness.input.requested_yaw_rad = sign * 5.0 * kDeg;
    auto output = harness.step();
    CHECK(output.adjusting && near(output.yaw_rad, sign * harness.config.step_rad));
    const double fixedTarget = output.yaw_rad;
    harness.input.allow_new_target = false;
    for (int tick = 1; tick <= 16; ++tick) {
      harness.input.yaw_rad = sign * tick * 0.167 * kDeg;
      output = harness.step(0.1);
      CHECK(output.adjusting && !output.step_timed_out);
      CHECK(near(output.yaw_rad, fixedTarget));
    }
    const double reachedYaw = sign * (harness.config.step_rad - 0.161 * kDeg);
    harness.input.yaw_rad = reachedYaw;
    output = harness.step(0.1);
    CHECK(!output.adjusting && !output.step_timed_out && output.waiting_for_capture);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));
    CHECK(near(output.step_motion_rad, reachedYaw));
    const double settledHold = output.yaw_rad;
    const auto stopStamp = harness.input.camera_now_ns;

    // Even after the pause, a camera capture from the stop instant is too old.
    output = harness.step(harness.config.observation_pause_sec + 0.01, false);
    CHECK(!output.adjusting && output.waiting_for_capture);
    harness.input.allow_new_target = true;
    harness.input.pose_stamp_ns = stopStamp;
    CHECK(harness.step(0.01, false).waiting_for_capture);

    // A later capture remains blocked until the docking controller permits a
    // new target. Small measured drift does not refresh the settled hold.
    harness.input.allow_new_target = false;
    harness.input.yaw_rad += sign * 0.167 * kDeg;
    output = harness.step(0.01);
    CHECK(!output.adjusting && output.waiting_for_capture);
    CHECK(near(output.yaw_rad, settledHold));
    harness.input.allow_new_target = true;
    output = harness.step(0.01, false);
    CHECK(output.adjusting && !output.waiting_for_capture);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad + sign * harness.config.step_rad));
  }

  // Blocking a new target at startup leaves the initial hold unchanged.
  YawHarness initiallyBlocked;
  initiallyBlocked.input.allow_new_target = false;
  auto output = initiallyBlocked.step(1.0);
  CHECK(!output.adjusting && !output.waiting_for_capture);
  CHECK(near(output.yaw_rad, 0.0));
  initiallyBlocked.input.allow_new_target = true;
  CHECK(initiallyBlocked.step().adjusting);
}

void testYawVelocityPulseDefaultsAndValidation() {
  const YawController::Config defaults;
  CHECK(near(defaults.step_rad, 3.0 * kDeg));
  CHECK(near(defaults.motor_velocity_rad_s, 3.5));
  CHECK(near(defaults.velocity_pulse_sec, 0.015));

  // Velocity authorization uses the motor command units from teleoperation,
  // independently of a tiny or lagging core request.
  for (double sign : {-1.0, 1.0}) {
    YawHarness harness;
    harness.input.requested_yaw_rad = sign * 0.01 * kDeg;
    harness.input.camera_x_error_m = sign * 0.05;
    auto output = harness.step();
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
    const double fixedTarget = output.yaw_rad;
    harness.input.allow_new_target = false;
    harness.input.yaw_rad = sign * 0.5 * kDeg;
    output = harness.step(defaults.velocity_pulse_sec + 0.01, false);
    CHECK(output.adjusting && near(output.yaw_rad, fixedTarget));
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));

    // A STOP/tracking interruption removes velocity authorization immediately.
    harness.input.allow_alignment = false;
    output = harness.step(0.001, false);
    CHECK(!output.adjusting && output.waiting_for_capture);
    CHECK(near(output.motor_velocity_rad_s, 0.0));
  }

  for (double speed : {0.0, -3.5, 5.001,
                       std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    YawController::Config config;
    config.motor_velocity_rad_s = speed;
    YawHarness invalid(config);
    const auto output = invalid.step();
    CHECK(output.failed && !output.adjusting);
    CHECK(near(output.motor_velocity_rad_s, 0.0));
  }
  for (double duration : {0.0, -0.01, 0.050001,
                          std::numeric_limits<double>::infinity()}) {
    YawController::Config config;
    config.velocity_pulse_sec = duration;
    YawHarness invalid(config);
    const auto output = invalid.step();
    CHECK(output.failed && !output.adjusting);
    CHECK(near(output.motor_velocity_rad_s, 0.0));
  }
  YawController::Config oversizedStep;
  oversizedStep.step_rad = 5.001 * kDeg;
  YawHarness invalidStep(oversizedStep);
  auto output = invalidStep.step();
  CHECK(output.failed && !output.adjusting);
  CHECK(near(output.motor_velocity_rad_s, 0.0));

  YawController::Config oversizedDeadband;
  oversizedDeadband.target_deadband_rad = 1.001 * kDeg;
  YawHarness invalidDeadband(oversizedDeadband);
  output = invalidDeadband.step();
  CHECK(output.failed && !output.adjusting);
  CHECK(near(output.motor_velocity_rad_s, 0.0));

  YawHarness invalidFeedback;
  CHECK(invalidFeedback.step().adjusting);
  invalidFeedback.input.yaw_rad = std::numeric_limits<double>::quiet_NaN();
  output = invalidFeedback.step();
  CHECK(output.failed && !output.adjusting);
  CHECK(near(output.motor_velocity_rad_s, 0.0));

  YawHarness reset;
  CHECK(reset.step().adjusting);
  reset.controller.reset();
  output = reset.step();
  CHECK(output.failed && !output.adjusting);
  CHECK(near(output.motor_velocity_rad_s, 0.0));
}

void testYawPostPulseFeedbackRefreshesBaseline() {
  // Hardware observes the latest CAN receipt after each monitored pulse. A
  // second movement is compared with that receipt, not the pre-pulse angle.
  for (double sign : {-1.0, 1.0}) {
    const double initialYaw = 489.52 * kDeg;
    YawHarness harness(YawController::Config{}, initialYaw);
    harness.input.camera_x_error_m = sign * 0.05;
    harness.input.requested_yaw_rad = initialYaw + sign * 2.0 * kDeg;
    auto output = harness.step();
    CHECK(output.adjusting);

    harness.input.yaw_rad = initialYaw + sign * 3.0 * kDeg;
    output = harness.observe(harness.config.velocity_pulse_sec);
    CHECK(!output.failed && !output.feedback_recovery_required && !output.feedback_jump_detected);
    harness.input.yaw_rad += sign * 0.3 * kDeg;
    output = harness.step(0.01, false);
    CHECK(!output.failed && !output.feedback_recovery_required && !output.feedback_jump_detected);
    CHECK(!output.adjusting && output.waiting_for_capture);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));

    // Subsequent real receipts continue updating the trusted feedback, even
    // when pitch centering blocks a new visual yaw correction.
    harness.input.allow_new_target = false;
    harness.input.yaw_rad += sign * 3.0 * kDeg;
    output = harness.observe();
    CHECK(!output.failed && !output.feedback_recovery_required);
    harness.input.yaw_rad += sign * 3.0 * kDeg;
    output = harness.observe();
    CHECK(!output.failed && !output.feedback_recovery_required);
  }
}

void testYawSmallRecoilPreservesCoarseTargetAndDeadline() {
  const double initialYaw = 510.4 * kDeg;
  for (double sign : {-1.0, 1.0}) {
    YawHarness harness(YawController::Config{}, initialYaw);
    harness.input.camera_x_error_m = sign * 0.12;
    auto output = harness.step();
    CHECK(output.adjusting && !output.feedback_recovery_required);
    const double fixedTarget = initialYaw + sign * 3.0 * kDeg;
    CHECK(near(output.yaw_rad, fixedTarget));
    CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
    harness.input.allow_new_target = false;
    const auto captureStamp = harness.input.pose_stamp_ns;

    // The motor can recoil against friction or backlash between pulses. A
    // fresh, ordinary-sized reply in the opposite direction is still valid
    // position feedback; it must not cancel this target or demand recovery.
    for (int recoil = 1; recoil <= 3; ++recoil) {
      harness.input.yaw_rad = initialYaw - sign * recoil * kDeg;
      output = harness.observe(0.01, false);
      CHECK(!output.failed && !output.feedback_recovery_required);
      CHECK(!output.feedback_jump_detected && output.adjusting);
      CHECK(!output.waiting_for_capture && near(output.yaw_rad, fixedTarget));
      // Reusing that accepted reply for the command tick cannot invent a new
      // target, restart an observation, or reduce the working movement speed.
      output = harness.step(0.1, false, false);
      CHECK(output.adjusting && !output.failed && !output.feedback_recovery_required);
      CHECK(!output.waiting_for_capture && !output.step_timed_out);
      CHECK(near(output.yaw_rad, fixedTarget));
      CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
      CHECK(near(output.step_motion_rad, -sign * recoil * kDeg));
      CHECK(harness.input.pose_stamp_ns == captureStamp);
    }

    // Continue toward the original target with no new camera capture. Only
    // actually crossing that target starts the normal stopped observation.
    harness.input.yaw_rad = initialYaw;
    output = harness.step(0.1, false);
    CHECK(output.adjusting && !output.feedback_recovery_required);
    CHECK(near(output.yaw_rad, fixedTarget));
    harness.input.yaw_rad = initialYaw + sign * kDeg;
    output = harness.step(0.1, false);
    CHECK(output.adjusting && near(output.yaw_rad, fixedTarget));
    harness.input.yaw_rad = initialYaw + sign * 3.2 * kDeg;
    output = harness.step(0.1, false);
    CHECK(!output.adjusting && output.waiting_for_capture);
    CHECK(!output.failed && !output.feedback_recovery_required && !output.step_timed_out);
    CHECK(near(output.yaw_rad, harness.input.yaw_rad));
    CHECK(near(output.step_motion_rad, sign * 3.2 * kDeg));

    // Recoil also cannot extend a stalled correction beyond its original
    // two-second deadline, even if new-target creation stays blocked.
    YawHarness stalled(YawController::Config{}, initialYaw);
    stalled.input.camera_x_error_m = sign * 0.12;
    output = stalled.step();
    const double stepStartTime = stalled.input.now_sec;
    stalled.input.allow_new_target = false;
    for (int recoil = 1; recoil <= 4; ++recoil) {
      stalled.input.yaw_rad = initialYaw - sign * recoil * kDeg;
      output = stalled.observe(0.01, false);
      CHECK(output.adjusting && !output.feedback_recovery_required);
      output = stalled.step(0.39, false, false);
      CHECK(output.adjusting && !output.step_timed_out);
      CHECK(!output.feedback_recovery_required && !output.waiting_for_capture);
      CHECK(near(output.yaw_rad, fixedTarget));
      CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
    }
    stalled.input.yaw_rad = initialYaw - sign * 5.0 * kDeg;
    output = stalled.observe(0.01, false);
    CHECK(output.adjusting && !output.feedback_recovery_required);
    const double remaining = stalled.config.adjustment_timeout_sec -
        (stalled.input.now_sec - stepStartTime);
    output = stalled.step(remaining + 0.01, false, false);
    CHECK(!output.adjusting && output.step_timed_out && output.waiting_for_capture);
    CHECK(!output.failed && !output.feedback_recovery_required);
    CHECK(near(output.yaw_rad, stalled.input.yaw_rad));
    CHECK(near(output.step_rad, sign * 3.0 * kDeg));
    CHECK(near(output.step_motion_rad, -sign * 5.0 * kDeg));
  }
}

void testYawJumpRecoveryNeedsDistinctCanReceiptsAndCapture() {
  const double initialYaw = 489.52 * kDeg;
  YawHarness harness(YawController::Config{}, initialYaw);
  CHECK(harness.step().adjusting);
  harness.input.yaw_rad = initialYaw + 5.1 * kDeg;
  auto output = harness.step();
  CHECK(output.feedback_recovery_required && output.feedback_jump_detected && !output.failed);
  CHECK(!output.adjusting && near(output.motor_velocity_rad_s, 0.0));
  CHECK(near(output.feedback_jump_rad, 5.1 * kDeg));
  CHECK(near(output.yaw_rad, initialYaw));

  // A cached CAN reading and arbitrarily many camera captures are not new
  // motor evidence. Recovery must not confirm the angle from repeated ticks.
  for (int tick = 0; tick < 5; ++tick) {
    output = harness.step(0.25, true, false);
    CHECK(output.feedback_recovery_required && !output.failed && !output.adjusting);
    CHECK(!output.feedback_jump_detected);
    CHECK(near(output.yaw_rad, initialYaw));
    CHECK(near(output.motor_velocity_rad_s, 0.0));
  }

  harness.input.yaw_rad = initialYaw + 5.2 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, initialYaw));
  // Repeating the first agreeing receipt cannot serve as the second one.
  output = harness.observe(0.25, true, false);
  CHECK(output.feedback_recovery_required && near(output.yaw_rad, initialYaw));

  harness.input.yaw_rad = initialYaw + 5.1 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  const auto confirmationStamp = harness.input.camera_now_ns;
  // A fresh capture cannot skip the observation pause, and the pause alone
  // cannot reuse a camera image captured before the angle was confirmed.
  harness.input.pose_stamp_ns = confirmationStamp + 1;
  output = harness.step(harness.config.observation_pause_sec / 2.0, false);
  CHECK(output.feedback_recovery_required && !output.adjusting);
  harness.input.pose_stamp_ns = confirmationStamp;
  output = harness.step(harness.config.observation_pause_sec, false);
  CHECK(output.feedback_recovery_required && !output.adjusting);
  output = harness.step(0.01, true);
  CHECK(!output.feedback_recovery_required && !output.failed);
}

void testYawTransientLargeBurstChangeUsesFeedbackRecovery() {
  YawHarness harness;
  harness.input.require_recovery_capture = false;
  CHECK(harness.step().adjusting);
  // The pulse observed a >5-degree transient, but the stopped reply has
  // returned near the original angle. Confirmation must still be required.
  harness.input.yaw_rad = -0.7 * kDeg;
  harness.input.force_feedback_confirmation = true;
  auto output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  CHECK(!output.adjusting && near(output.motor_velocity_rad_s, 0.0));
  CHECK(near(output.yaw_rad, 0.0));
  harness.input.force_feedback_confirmation = false;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.observe();
  CHECK(!output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
}

void testYawOneOffBadFrameAndConfirmedLargeHeading() {
  const double initialYaw = 510.4 * kDeg;
  YawHarness harness(YawController::Config{}, initialYaw);
  harness.input.allow_alignment = false;
  harness.input.require_recovery_capture = false;
  harness.input.yaw_rad = initialYaw + 40.0 * kDeg;
  auto output = harness.observe();
  CHECK(!output.failed && output.feedback_recovery_required && output.feedback_jump_detected);
  CHECK(near(output.yaw_rad, initialYaw));

  // Returning from a single implausible frame changes the recovery candidate;
  // two further agreeing receipts establish the original angle again.
  harness.input.yaw_rad = initialYaw;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.observe();
  CHECK(!output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, initialYaw));

  // A later finite heading 40 degrees away can be accepted after two new
  // agreeing CAN replies. Displacement from docking start is not a failure.
  harness.input.yaw_rad = initialYaw + 40.0 * kDeg;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.observe();
  CHECK(!output.failed && !output.feedback_recovery_required && !output.adjusting);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  CHECK(near(output.motor_velocity_rad_s, 0.0));
}

void testYawRecoveryCanOnlyResumeAndUnstableFramesStayStopped() {
  YawHarness harness;
  harness.input.require_recovery_capture = false;
  harness.input.allow_alignment = false;
  harness.input.allow_new_target = false;
  harness.input.yaw_rad = 6.0 * kDeg;
  auto output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required && !output.failed);
  harness.input.yaw_rad = 6.1 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required);
  harness.input.yaw_rad = 8.0 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, 0.0));
  harness.input.yaw_rad = 8.1 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(output.feedback_recovery_required);
  harness.input.yaw_rad = 8.2 * kDeg;
  output = harness.observe(0.01, false);
  CHECK(!output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  CHECK(!output.adjusting && near(output.motor_velocity_rad_s, 0.0));
  // Port preparation/insertion can recover CAN feedback with no new camera
  // capture, and cannot accidentally enable visual yaw pulses.
  output = harness.step(0.25, false);
  CHECK(!output.feedback_recovery_required && !output.failed && !output.adjusting);
  CHECK(near(output.motor_velocity_rad_s, 0.0));
}

void testYawRecoveryRejectsDriftingConfirmationChain() {
  YawHarness harness;
  harness.input.require_recovery_capture = false;
  harness.input.allow_alignment = false;
  harness.input.yaw_rad = 5.1 * kDeg;
  auto output = harness.observe();
  CHECK(output.feedback_recovery_required && output.feedback_jump_detected);
  harness.input.yaw_rad = 6.0 * kDeg;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  harness.input.yaw_rad = 6.9 * kDeg;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, 0.0));
  CHECK(!output.adjusting && near(output.motor_velocity_rad_s, 0.0));

  // Each adjacent change was less than one degree, but the confirming replies
  // must agree with one fixed candidate, not a moving previous-frame anchor.
  harness.input.yaw_rad = 7.0 * kDeg;
  output = harness.observe();
  CHECK(output.feedback_recovery_required && !output.failed);
  harness.input.yaw_rad = 6.9 * kDeg;
  output = harness.observe();
  CHECK(!output.feedback_recovery_required && !output.failed);
  CHECK(near(output.yaw_rad, harness.input.yaw_rad));
  CHECK(!output.adjusting && near(output.motor_velocity_rad_s, 0.0));
}

void testDockingYawHoldResynchronizationAcceptsAnyFiniteHeading() {
  for (Phase phase : {Phase::SEARCH, Phase::INSERT}) {
    Harness harness;
    const double originalYaw = harness.input.yaw_rad;
    harness.reach(phase);
    CHECK(harness.controller.phase() == phase);

    const double recoveredYaw = originalYaw + 40.0 * kDeg;
    CHECK(harness.controller.resynchronizeYawHold(recoveredYaw));
    harness.input.yaw_rad = recoveredYaw;
    auto output = harness.step();
    CHECK(output.phase == phase);
    CHECK(near(output.yaw_rad, recoveredYaw));

    // Invalid feedback still cannot replace the confirmed finite hold target.
    CHECK(!harness.controller.resynchronizeYawHold(
        std::numeric_limits<double>::quiet_NaN()));
    CHECK(!harness.controller.resynchronizeYawHold(
        std::numeric_limits<double>::infinity()));
    CHECK(!harness.controller.resynchronizeYawHold(
        -std::numeric_limits<double>::infinity()));
    output = harness.step();
    CHECK(output.phase == phase);
    CHECK(near(output.yaw_rad, recoveredYaw));

    const double reverseRecoveredYaw = originalYaw - 40.0 * kDeg;
    CHECK(harness.controller.resynchronizeYawHold(reverseRecoveredYaw));
    harness.input.yaw_rad = reverseRecoveredYaw;
    output = harness.step();
    CHECK(output.phase == phase);
    CHECK(near(output.yaw_rad, reverseRecoveredYaw));
    if (phase == Phase::INSERT) {
      CHECK(output.actuation == Actuation::INSERT_HOLD_PITCH);
      CHECK(near(output.zipper_motor_velocity_rad_s, harness.config.insertion_velocity_rad_s));
    } else {
      CHECK(output.actuation == Actuation::TRACK_POSE);
      CHECK(near(output.extension_m, harness.input.extension_m));
    }
  }
}

}  // namespace

int main() {
  try {
    testSearch();
    testPitchCenteringAndHolding();
    testAdaptivePitchCoarseFineAndDirections();
    testAdaptivePitchSmoothTransition();
    testAdaptivePitchRemainingErrorAndReversal();
    testAdaptivePitchConfiguration();
    testInsertionHandoff();
    testInsertionPriorityAndFastHook();
    testCloseRangeHandoffRegression();
    testDockPreparationOrderAndCapture();
    testDockPreparationRequiresFlags();
    testDockPreparationFreshnessAndTimeout();
    testDockPreparationContactPriority();
    testDockPreparationTimingValidation();
    testProductionDockCommandDwell();
    testContactSupersedesQueuedOpenAndSpacesLatchCommands();
    testQueuedDockPreparationStartsTimeoutAtEmission();
    testDockRetryCommandsRespectSpacing();
    testDockCommandIntervalValidation();
    testDockInsertionHeartbeatRecovery();
    testDockInsertionNotReadyRecovery();
    testDockDeactivationAcknowledgement();
    testDockPreparationTimeoutRetry();
    testDockRecoveryRequiresSafeFreshState();
    testDockRetryBudgetAndHeldPose();
    testLatchRecoveryPreservesContactContext();
    testLatchCycleRejectsRetainedCompleteAndCachedState();
    testLatchClosingRequiresFreshPowerAndPulse();
    testLatchRecoveryTimeoutAndNewReceipt();
    testLatchRetryBudgetIndependentOfApproach();
    testLatchConfiguration();
    testDockRetryConfiguration();
    testInsertionUntilContact();
    testApproachBurstAndNewCapture();
    testTrackingPauseAndResume();
    testContactPriority();
    testDockFreshnessAndLatchTimeout();
    testValidationResetAndBurstWait();
    testYawAlignmentPhaseGuard();
    testYawThreeCentimeterLateralDeadband();
    testYawSmallRequestsFullStep();
    testYawCameraDirectionSurvivesLaggingCoreTarget();
    testYawCumulativeSmallStepsContinueBeyondThirtyDegrees();
    testYawTimeoutMotionReporting();
    testYawStepsObservationAndDirection();
    testYawCancellationSettledHoldAndValidation();
    testYawCaptureWaitPreservesPositionTargetAndDeadline();
    testYawQuantizedFeedbackAndBlockedObservation();
    testYawVelocityPulseDefaultsAndValidation();
    testYawPostPulseFeedbackRefreshesBaseline();
    testYawSmallRecoilPreservesCoarseTargetAndDeadline();
    testYawJumpRecoveryNeedsDistinctCanReceiptsAndCapture();
    testYawTransientLargeBurstChangeUsesFeedbackRecovery();
    testYawOneOffBadFrameAndConfirmedLargeHeading();
    testYawRecoveryCanOnlyResumeAndUnstableFramesStayStopped();
    testYawRecoveryRejectsDriftingConfirmationChain();
    testDockingYawHoldResynchronizationAcceptsAnyFiniteHeading();
    std::cout << "All autonomous docking controller tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Autonomous docking test failed: " << error.what() << '\n';
    return 1;
  }
}
