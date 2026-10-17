#include "docking_yaw_control.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::DockingYawController;
constexpr double kDeg = 0.017453292519943295;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(double actual, double expected, double tolerance = 1e-9) {
  return std::fabs(actual - expected) <= tolerance;
}

struct Harness {
  Controller controller;
  Controller::Input input;

  explicit Harness(Controller::Config config = Controller::Config{}) : controller(config) {
    input.camera_now_ns = input.pose_stamp_ns = 1000000000;
    input.feedback_sequence = 1;
    input.camera_x_error_m = 0.1;
    input.camera_z_m = 0.8;
    input.allow_alignment = true;
    controller.start(0.0, 0.0, input.feedback_sequence);
  }

  Controller::Output step(double dt = 0.01, bool new_capture = true,
                          bool new_feedback = true) {
    input.now_sec += dt;
    input.camera_now_ns += static_cast<std::int64_t>(dt * 1e9);
    if (new_capture) input.pose_stamp_ns = input.camera_now_ns;
    if (new_feedback) ++input.feedback_sequence;
    const auto output = controller.update(input);
    if (output.adjusting && !output.feedback_recovery_required && !output.failed) {
      CHECK(near(std::fabs(output.motor_velocity_rad_s), 3.5));
      CHECK(output.velocity_pulse_sec > 0.0);
    } else {
      CHECK(near(output.motor_velocity_rad_s, 0.0));
    }
    return output;
  }
};

void testDepthScalesNewTargetsWithoutReducingBreakawayVelocity() {
  struct Example { double depth_m; double step_deg; double pulse_sec; };
  const Example examples[] = {
      {0.8, 3.0, 0.015}, {0.45, 3.0, 0.015},
      {0.35, 2.25, 0.0115}, {0.25, 1.5, 0.008},
      {0.20, 1.5, 0.008}, {0.0, 3.0, 0.015}};
  for (double sign : {-1.0, 1.0}) {
    for (const auto& example : examples) {
      Harness harness;
      harness.input.camera_z_m = example.depth_m;
      harness.input.camera_x_error_m = sign * 0.1;
      const auto output = harness.step();
      CHECK(output.adjusting && !output.failed);
      CHECK(near(output.step_rad, sign * example.step_deg * kDeg));
      CHECK(near(output.velocity_pulse_sec, example.pulse_sec));
      CHECK(near(output.motor_velocity_rad_s, sign * 3.5));
    }
  }
}

void testActiveTargetPulseAndDeadlineAreFrozenAcrossDepthUpdates() {
  for (double sign : {-1.0, 1.0}) {
    Harness harness;
    harness.input.camera_z_m = 0.35;
    harness.input.camera_x_error_m = sign * 0.1;
    const auto original = harness.step();
    CHECK(original.adjusting);
    harness.input.camera_z_m = 0.20;
    harness.input.yaw_rad = sign * 0.4 * kDeg;
    auto output = harness.step(0.5);
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.yaw_rad, original.yaw_rad));
    CHECK(near(output.step_rad, original.step_rad));
    CHECK(near(output.velocity_pulse_sec, original.velocity_pulse_sec));
    harness.input.camera_z_m = 0.8;
    output = harness.step(1.49);
    CHECK(output.adjusting && near(output.yaw_rad, original.yaw_rad));
    CHECK(near(output.velocity_pulse_sec, original.velocity_pulse_sec));
    output = harness.step(0.011);
    CHECK(!output.adjusting && !output.failed && output.step_timed_out);
    CHECK(output.waiting_for_capture);

    // A new target, after its observation pause, uses the new depth profile.
    harness.input.camera_z_m = 0.25;
    output = harness.step(0.21);
    CHECK(output.adjusting && !output.failed);
    CHECK(near(output.step_rad, sign * 1.5 * kDeg));
    CHECK(near(output.velocity_pulse_sec, 0.008));
  }
}

void testNearApproachReopensCoarseAlignmentAndKeepsItsHysteresis() {
  for (double sign : {-1.0, 1.0}) {
    Harness harness;
    harness.input.camera_x_error_m = sign * 0.025;
    CHECK(!harness.step().adjusting);  // Far alignment accepts 2.5 cm.
    harness.input.camera_z_m = 0.44;
    auto output = harness.step();
    CHECK(output.adjusting && !output.failed);  // Near goal requires 1 cm.
    harness.input.camera_x_error_m = sign * 0.015;
    output = harness.step();
    CHECK(output.adjusting);  // Not yet aligned; the 2 cm restart gate cannot stop it.
    harness.input.camera_x_error_m = sign * 0.010;
    output = harness.step();
    CHECK(!output.adjusting && output.waiting_for_capture);
    const double held_yaw = output.yaw_rad;
    for (double lateral_error : {sign * 0.015, -sign * 0.019, sign * 0.020}) {
      harness.input.camera_x_error_m = lateral_error;
      // A depth estimate moving across the near boundary must not reset the
      // aligned state or loosen the lateral goal back to the coarse window.
      harness.input.camera_z_m = 0.8;
      output = harness.step(0.25);
      CHECK(!output.adjusting && !output.waiting_for_capture && !output.failed);
      CHECK(near(output.yaw_rad, held_yaw));
    }
    harness.input.camera_x_error_m = sign * 0.021;
    output = harness.step();
    CHECK(output.adjusting && !output.failed);  // Still the near 2 cm restart gate.

    // Restarting docking clears the latched near mode and alignment state.
    harness.controller.start(harness.input.now_sec, harness.input.yaw_rad,
                             harness.input.feedback_sequence);
    harness.input.camera_x_error_m = sign * 0.025;
    output = harness.step();
    CHECK(!output.adjusting && !output.failed);
  }
}

void testPausedOrUnstampedVisionCannotLatchNearAlignment() {
  Harness paused;
  paused.input.allow_alignment = false;
  paused.input.camera_z_m = 0.25;
  paused.input.camera_x_error_m = 0.025;
  CHECK(!paused.step().adjusting);
  paused.input.allow_alignment = true;
  paused.input.camera_z_m = 0.8;
  CHECK(!paused.step().adjusting);

  Harness unstamped;
  unstamped.input.camera_z_m = 0.25;
  unstamped.input.camera_x_error_m = 0.025;
  unstamped.input.pose_stamp_ns = 0;
  CHECK(!unstamped.step(0.01, false).adjusting);
  CHECK(unstamped.step().adjusting);
}

void testFeedbackRecoveryCannotEnableNearAlignmentBeforeVisionResumes() {
  Harness harness;
  harness.input.require_recovery_capture = true;
  harness.input.camera_x_error_m = 0.025;
  CHECK(!harness.step().adjusting);
  harness.input.yaw_rad = 6.0 * kDeg;
  harness.input.camera_z_m = 0.25;
  auto output = harness.step();
  CHECK(output.feedback_recovery_required && !output.failed);
  output = harness.step();
  CHECK(output.feedback_recovery_required);
  harness.input.camera_z_m = 0.8;
  output = harness.step();
  CHECK(output.feedback_recovery_required);
  output = harness.step(0.25);
  CHECK(!output.feedback_recovery_required && !output.adjusting && !output.failed);
  // The near pose seen during CAN recovery did not tighten the far goal.
  CHECK(near(output.motor_velocity_rad_s, 0.0));
  harness.input.camera_z_m = 0.25;
  CHECK(harness.step().adjusting);
}

void testSmallRecoilAndTargetCrossingDoNotReenterRecovery() {
  for (double sign : {-1.0, 1.0}) {
    Harness harness;
    harness.input.camera_z_m = 0.25;
    harness.input.camera_x_error_m = sign * 0.1;
    CHECK(harness.step().adjusting);
    harness.input.yaw_rad = -sign * 0.8 * kDeg;
    auto output = harness.step();
    CHECK(output.adjusting && !output.failed && !output.feedback_recovery_required);
    CHECK(near(output.yaw_rad, sign * 1.5 * kDeg));
    harness.input.yaw_rad = sign * 1.7 * kDeg;
    output = harness.step();
    CHECK(!output.adjusting && !output.failed && !output.feedback_recovery_required);
    CHECK(output.waiting_for_capture);

    harness.input.yaw_rad += sign * 6.0 * kDeg;
    output = harness.step();
    CHECK(output.feedback_recovery_required && output.feedback_jump_detected);
    CHECK(!output.failed);  // The existing large-jump recovery remains intact.
  }
}

void rejectsConfig(Controller::Config config) {
  Harness harness(config);
  const auto output = harness.step();
  CHECK(output.failed && !output.adjusting);
}

void testInvalidNearProfilesAndDepthInputsAreRejected() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  for (double step : {0.0, 0.1 * kDeg, 0.2 * kDeg, 3.01 * kDeg, nan, infinity}) {
    Controller::Config config;
    config.near_step_rad = step;
    rejectsConfig(config);
  }
  for (double pulse : {0.0, -0.001, 0.01501, nan, infinity}) {
    Controller::Config config;
    config.near_velocity_pulse_sec = pulse;
    rejectsConfig(config);
  }
  for (double stop : {-0.001, 0.031, nan, infinity}) {
    Controller::Config config;
    config.near_camera_lateral_deadband_m = stop;
    rejectsConfig(config);
  }
  for (double resume : {-0.001, 0.009, 0.051, nan, infinity}) {
    Controller::Config config;
    config.near_camera_lateral_resume_m = resume;
    rejectsConfig(config);
  }
  Controller::Config reversed_profile;
  reversed_profile.approach_profile.slowdown_end_m = 0.5;
  rejectsConfig(reversed_profile);
  for (double depth : {-0.001, nan, infinity}) {
    Harness harness;
    harness.input.camera_z_m = depth;
    const auto output = harness.step();
    CHECK(output.failed && !output.adjusting);
  }
}
}  // namespace

int main() {
  try {
    testDepthScalesNewTargetsWithoutReducingBreakawayVelocity();
    testActiveTargetPulseAndDeadlineAreFrozenAcrossDepthUpdates();
    testNearApproachReopensCoarseAlignmentAndKeepsItsHysteresis();
    testPausedOrUnstampedVisionCannotLatchNearAlignment();
    testFeedbackRecoveryCannotEnableNearAlignmentBeforeVisionResumes();
    testSmallRecoilAndTargetCrossingDoNotReenterRecovery();
    testInvalidNearProfilesAndDepthInputsAreRejected();
    std::cout << "Near docking yaw tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
