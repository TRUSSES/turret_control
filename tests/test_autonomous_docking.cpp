#include "autonomous_docking.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::AutonomousDockingController;
using Phase = Controller::Phase;
using Actuation = Controller::Actuation;
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

struct Harness {
  Controller::Config config;
  Controller controller;
  Controller::Input input;

  explicit Harness(Controller::Config settings = Controller::Config{})
      : config(settings), controller(settings) {
    input.extension_m = 0.25;
    input.pitch_rad = 20.0 * kDeg;
    input.yaw_rad = 0.1;
    input.dock_fresh = true;
    input.camera_y_m = settings.goal_camera_y_m;
    input.camera_z_m = 0.5;
    controller.start(input);
  }

  Controller::Output step(double dt = 0.1) {
    input.now_sec += dt;
    return controller.update(input);
  }

  void follow(const Controller::Output& output) {
    input.extension_m = output.extension_m;
    input.pitch_rad = output.pitch_rad;
    input.yaw_rad = output.yaw_rad;
  }

  Controller::Output align() {
    input.pose_fresh = true;
    input.camera_x_m = 0.04;
    return step();
  }

  Controller::Output approach() {
    align();
    input.camera_x_m = config.goal_camera_x_m;
    input.camera_y_m = config.goal_camera_y_m;
    return step();
  }

  Controller::Output insert() {
    approach();
    input.camera_z_m = config.insertion_distance_m;
    return step();
  }

  Controller::Output lose() {
    approach();
    input.pose_fresh = false;
    return step();
  }

  void reach(Phase phase) {
    if (phase == Phase::SEARCH) return;
    if (phase == Phase::ALIGN) { align(); return; }
    if (phase == Phase::APPROACH) { approach(); return; }
    if (phase == Phase::INSERT) { insert(); return; }
    if (phase == Phase::WAIT_FOR_LATCH) {
      insert();
      input.dock_detected = true;
      step();
      return;
    }
    lose();
    if (phase == Phase::RECOVERY_STOP) return;
    step();
    if (phase == Phase::RECOVERY_RETRACT) return;
    input.extension_m -= config.recovery_retract_m;
    step();
    CHECK(controller.phase() == Phase::RECOVERY_SWEEP);
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

  // Commands at 100 Hz must eventually cross the real pitch controller's
  // 0.5 degree deadband even when measured pitch has not yet changed.
  Harness fast;
  for (int index = 0; index < 20; ++index) output = fast.step(0.01);
  CHECK(output.pitch_rad - fast.input.pitch_rad > 0.5 * kDeg);
  CHECK(output.pitch_rad - fast.input.pitch_rad <= fast.config.max_pitch_step_rad + 1e-9);
  for (int index = 0; index < 1000; ++index) output = fast.step(0.01);
  CHECK(output.pitch_rad - fast.input.pitch_rad <= fast.config.max_pitch_step_rad + 1e-9);

  Harness delayed;
  output = delayed.step(10.0);
  CHECK(output.pitch_rad - delayed.input.pitch_rad <=
        delayed.config.search_pitch_rate_rad_s * delayed.config.max_dt_sec + 1e-9);
  output = delayed.step(10.0);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Controller::Config smallSpan;
  smallSpan.search_pitch_span_rad = 1.0 * kDeg;
  Harness bounded(smallSpan);
  for (int index = 0; index < 30; ++index) {
    output = bounded.step();
    bounded.follow(output);
  }
  CHECK(near(bounded.input.pitch_rad, 21.0 * kDeg));
}

void testAlignment() {
  Harness harness;
  harness.input.camera_y_m = 0.08;
  auto output = harness.align();
  CHECK(output.phase == Phase::ALIGN);
  CHECK(output.yaw_rad > harness.input.yaw_rad);
  CHECK(output.pitch_rad < harness.input.pitch_rad);
  CHECK(near(output.extension_m, harness.input.extension_m));

  Controller::Config reverse;
  reverse.yaw_correction_sign = -1.0;
  Harness reversed(reverse);
  CHECK(reversed.align().yaw_rad < reversed.input.yaw_rad);

  Harness deadband;
  deadband.input.pose_fresh = true;
  deadband.input.camera_x_m = 0.003;
  deadband.input.camera_y_m = deadband.config.goal_camera_y_m + 0.003;
  output = deadband.step();
  CHECK(near(output.yaw_rad, deadband.input.yaw_rad));
  CHECK(near(output.pitch_rad, deadband.input.pitch_rad));

  Harness clamped;
  clamped.input.pitch_rad = clamped.config.pitch_max_rad - 0.005;
  clamped.controller.start(clamped.input);
  clamped.input.pose_fresh = true;
  clamped.input.camera_x_m = 1.0;
  clamped.input.camera_y_m = -1.0;
  const double startingYaw = clamped.input.yaw_rad;
  for (int index = 0; index < 100; ++index) {
    output = clamped.step();
    CHECK(output.phase == Phase::ALIGN);
    CHECK(output.pitch_rad <= clamped.config.pitch_max_rad + 1e-9);
    CHECK(output.yaw_rad <= startingYaw + clamped.config.yaw_limit_from_start_rad + 1e-9);
    CHECK(std::fabs(output.yaw_rad - clamped.input.yaw_rad) <=
          clamped.config.max_yaw_rate_rad_s * 0.1 + 1e-9);
    clamped.follow(output);
  }
  CHECK(near(output.pitch_rad, clamped.config.pitch_max_rad));
  CHECK(near(output.yaw_rad, startingYaw + clamped.config.yaw_limit_from_start_rad));
}

void testApproachAndInsertionHandoff() {
  Harness harness;
  auto output = harness.approach();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.extension_m > harness.input.extension_m);
  CHECK(output.extension_m - harness.input.extension_m <=
        harness.config.max_extension_rate_m_s * 0.1 + 1e-9);
  harness.follow(output);
  harness.input.camera_z_m = 0.11;
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  harness.follow(output);

  harness.input.camera_z_m = 0.10;
  harness.input.camera_x_m = 0.02;
  output = harness.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.extension_m, harness.input.extension_m));
  CHECK(output.yaw_rad > harness.input.yaw_rad);

  harness.input.camera_x_m = 0.01;
  output = harness.step();
  CHECK(output.phase == Phase::INSERT);
  CHECK(output.actuation == Actuation::INSERT_FREE_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.3));

  Harness missingDock;
  missingDock.approach();
  missingDock.input.camera_z_m = 0.10;
  missingDock.input.dock_fresh = false;
  output = missingDock.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness maxExtension;
  maxExtension.approach();
  maxExtension.input.extension_m = maxExtension.config.extension_max_m;
  output = maxExtension.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness fast;
  fast.approach();
  for (int index = 0; index < 10; ++index) output = fast.step(0.01);
  CHECK(output.extension_m - fast.input.extension_m > 0.001);
  CHECK(output.extension_m - fast.input.extension_m <= fast.config.max_extension_step_m + 1e-9);

  Harness depthDeadband;
  depthDeadband.approach();
  depthDeadband.input.camera_z_m = 0.103;
  CHECK(depthDeadband.step().phase == Phase::INSERT);
}

void testInsertionUntilContact() {
  Harness harness;
  auto output = harness.insert();
  const double heldYaw = output.yaw_rad;
  harness.input.pose_fresh = false;
  harness.input.camera_z_m = std::numeric_limits<double>::quiet_NaN();
  for (double extension : {0.9, 2.0, 100.0}) {
    harness.input.extension_m = extension;
    harness.input.yaw_rad += 0.01;
    output = harness.step(500.0);
    CHECK(output.phase == Phase::INSERT);
    CHECK(output.actuation == Actuation::INSERT_FREE_PITCH);
    CHECK(near(output.extension_m, extension));
    CHECK(near(output.yaw_rad, heldYaw));
    CHECK(output.zipper_motor_velocity_rad_s > 0.0);
  }
  harness.input.dock_detected = true;
  output = harness.step();
  CHECK(output.phase == Phase::WAIT_FOR_LATCH);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  const auto contactPose = output;
  harness.input.extension_m += 0.001;
  output = harness.step();
  CHECK(near(output.extension_m, contactPose.extension_m));
  harness.input.dock_complete = true;
  output = harness.step();
  CHECK(output.phase == Phase::COMPLETE);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(harness.step(100.0).phase == Phase::COMPLETE);
}

void testRecoverySequence() {
  Harness harness;
  auto output = harness.lose();
  CHECK(output.phase == Phase::RECOVERY_STOP);
  CHECK(output.actuation == Actuation::STOP);
  const double lostExtension = harness.input.extension_m;
  const double lostPitch = harness.input.pitch_rad;
  const double lostYaw = harness.input.yaw_rad;

  harness.input.pose_fresh = true;  // Reappearing tag must not skip safe retraction.
  output = harness.step();
  CHECK(output.phase == Phase::RECOVERY_RETRACT);
  CHECK(output.actuation == Actuation::TRACK_POSE);
  CHECK(output.extension_m < lostExtension);
  CHECK(near(output.pitch_rad, lostPitch));
  CHECK(near(output.yaw_rad, lostYaw));
  output = harness.step();
  CHECK(output.phase == Phase::RECOVERY_RETRACT);

  harness.input.pose_fresh = false;
  harness.input.extension_m = lostExtension - harness.config.recovery_retract_m;
  output = harness.step();
  CHECK(output.phase == Phase::RECOVERY_SWEEP);
  CHECK(output.pitch_rad > lostPitch);
  CHECK(near(output.yaw_rad, lostYaw));
  CHECK(near(output.extension_m, harness.input.extension_m));

  // The real pitch actuator stops inside its 0.01 rad deadband. A sweep must
  // reverse before the encoder reaches the exact endpoint, even at 100 Hz.
  harness.input.pitch_rad = lostPitch + harness.config.recovery_pitch_span_rad - 0.5 * kDeg;
  for (int index = 0; index < 100; ++index) output = harness.step(0.01);
  CHECK(output.pitch_rad < harness.input.pitch_rad);
  harness.input.pitch_rad = lostPitch - harness.config.recovery_pitch_span_rad + 0.5 * kDeg;
  for (int index = 0; index < 100; ++index) output = harness.step(0.01);
  CHECK(output.pitch_rad > harness.input.pitch_rad);

  harness.input.pose_fresh = true;
  harness.input.camera_x_m = 0.04;
  output = harness.step();
  CHECK(output.phase == Phase::ALIGN);
  CHECK(output.actuation == Actuation::TRACK_POSE);
  CHECK(near(output.extension_m, harness.input.extension_m));
}

void testRecoveryBoundsAndTimeouts() {
  Controller::Config singleRetry;
  singleRetry.max_recovery_attempts = 1;
  Harness harness(singleRetry);
  harness.reach(Phase::RECOVERY_SWEEP);
  harness.input.pose_fresh = true;
  CHECK(harness.step().phase == Phase::ALIGN);
  harness.input.pose_fresh = false;
  auto output = harness.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness retractTimeout;
  retractTimeout.reach(Phase::RECOVERY_RETRACT);
  output = retractTimeout.step(retractTimeout.config.recovery_retract_timeout_sec);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness sweepTimeout;
  sweepTimeout.reach(Phase::RECOVERY_SWEEP);
  output = sweepTimeout.step(sweepTimeout.config.recovery_sweep_timeout_sec);
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);

  Harness nearZero;
  nearZero.input.extension_m = 0.01;
  nearZero.controller.start(nearZero.input);
  nearZero.lose();
  for (int index = 0; index < 3; ++index) {
    output = nearZero.step();
    CHECK(output.extension_m >= nearZero.config.extension_min_m);
    nearZero.follow(output);
  }

  Harness closeLoss;
  closeLoss.align();
  closeLoss.input.camera_z_m = 0.103;
  closeLoss.step();  // Still lateral error, so not insertion.
  closeLoss.input.pose_fresh = false;
  output = closeLoss.step();
  CHECK(output.phase == Phase::FAILED);
  CHECK(output.actuation == Actuation::STOP);
}

void testContactPriority() {
  const Phase phases[] = {Phase::SEARCH, Phase::ALIGN, Phase::APPROACH,
    Phase::RECOVERY_STOP, Phase::RECOVERY_RETRACT, Phase::RECOVERY_SWEEP,
    Phase::INSERT, Phase::WAIT_FOR_LATCH};
  for (Phase phase : phases) {
    Harness harness;
    harness.reach(phase);
    CHECK(harness.controller.phase() == phase);
    harness.input.dock_detected = true;
    harness.input.pose_fresh = false;
    auto output = harness.step();
    CHECK(output.phase == Phase::WAIT_FOR_LATCH);
    CHECK(output.actuation == Actuation::STOP);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    harness.input.dock_complete = true;
    output = harness.step();
    CHECK(output.phase == Phase::COMPLETE);
    CHECK(output.actuation == Actuation::STOP);
  }

  Harness closing;
  closing.input.dock_closing = true;
  CHECK(closing.step().phase == Phase::WAIT_FOR_LATCH);
  Harness simultaneous;
  simultaneous.insert();
  simultaneous.input.dock_detected = true;
  simultaneous.input.dock_complete = true;
  CHECK(simultaneous.step().phase == Phase::COMPLETE);

  Harness timeoutContact;
  timeoutContact.input.dock_detected = true;
  // A contact arriving on the search deadline still stops and waits for latch.
  CHECK(timeoutContact.step(timeoutContact.config.search_timeout_sec).phase == Phase::WAIT_FOR_LATCH);
}

void testDockFreshnessAndLatchTimeout() {
  for (Phase phase : {Phase::INSERT, Phase::WAIT_FOR_LATCH}) {
    Harness harness;
    harness.reach(phase);
    harness.input.dock_fresh = false;
    harness.input.dock_complete = true;  // Stale complete must not create success.
    const auto output = harness.step();
    CHECK(output.phase == Phase::FAILED);
    CHECK(output.actuation == Actuation::STOP);
  }

  Harness timeout;
  timeout.reach(Phase::WAIT_FOR_LATCH);
  CHECK(timeout.step(timeout.config.latch_timeout_sec).phase == Phase::FAILED);

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
  }
}

void testValidationResetAndPhaseTimeouts() {
  Controller::Config invalid;
  invalid.yaw_gain = std::numeric_limits<double>::quiet_NaN();
  Harness badConfig(invalid);
  CHECK(badConfig.step().phase == Phase::FAILED);
  invalid = Controller::Config{};
  invalid.pitch_min_rad = invalid.pitch_max_rad;
  Harness badBounds(invalid);
  CHECK(badBounds.step().phase == Phase::FAILED);

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

  Harness alignTimeout;
  alignTimeout.align();
  CHECK(alignTimeout.step(alignTimeout.config.alignment_timeout_sec).phase == Phase::FAILED);
  Harness approachTimeout;
  approachTimeout.approach();
  CHECK(approachTimeout.step(approachTimeout.config.approach_timeout_sec).phase == Phase::FAILED);

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

}  // namespace

int main() {
  try {
    testSearch();
    testAlignment();
    testApproachAndInsertionHandoff();
    testInsertionUntilContact();
    testRecoverySequence();
    testRecoveryBoundsAndTimeouts();
    testContactPriority();
    testDockFreshnessAndLatchTimeout();
    testValidationResetAndPhaseTimeouts();
    std::cout << "All autonomous docking controller tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Autonomous docking test failed: " << error.what() << '\n';
    return 1;
  }
}
