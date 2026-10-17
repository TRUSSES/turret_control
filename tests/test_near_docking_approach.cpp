#include "autonomous_docking.h"
#include "docking_approach_profile.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::AutonomousDockingController;
using Profile = turret_control::DockingApproachProfile;
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

struct Fixture {
  Controller::Config config;
  Controller controller;
  Controller::Input input;

  explicit Fixture(Controller::Config settings = Controller::Config{})
      : config(settings), controller(settings) {
    input.extension_m = 0.256;
    input.pitch_rad = 20.0 * kDeg;
    input.yaw_rad = 0.2;
    input.camera_z_m = 0.5;
    input.camera_now_ns = input.pose_stamp_ns = 1000000000;
    input.dock_fresh = input.dock_idle = input.dock_unoccupied = true;
    input.dock_state_sequence = 1;
    controller.start(input);
  }

  void imageError(double error_rad) {
    input.pose_fresh = true;
    input.camera_y_m = config.goal_camera_y_m - input.camera_z_m * std::tan(error_rad);
  }

  Controller::Output step(double dt = 0.1, bool new_capture = true) {
    input.now_sec += dt;
    input.camera_now_ns += static_cast<std::int64_t>(dt * 1e9);
    if (new_capture) input.pose_stamp_ns = input.camera_now_ns;
    return controller.update(input);
  }

  Controller::Output approach(double depth_m) {
    input.camera_z_m = depth_m;
    imageError(0.0);
    return step();
  }

  Controller::Output atPitchLimit(double depth_m = 0.214,
                                  double pitch_deg = 69.6,
                                  double image_error_deg = 4.8) {
    input.camera_z_m = depth_m;
    input.pitch_rad = pitch_deg * kDeg;
    controller.start(input);
    imageError(image_error_deg * kDeg);
    return step();
  }
};

void testDistanceProfile() {
  Profile profile;
  CHECK(profile.valid());
  for (double unavailable : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    CHECK(near(profile.nearBlend(unavailable), 0.0));
    CHECK(near(profile.interpolate(1.5, 1.2, unavailable), 1.5));
    CHECK(!profile.nearRange(unavailable));
  }
  CHECK(near(profile.nearBlend(0.60), 0.0));
  CHECK(near(profile.nearBlend(0.45), 0.0));
  CHECK(near(profile.nearBlend(0.35), 0.5));
  CHECK(near(profile.nearBlend(0.25), 1.0));
  CHECK(near(profile.nearBlend(0.21), 1.0));
  CHECK(near(profile.interpolate(1.5, 1.2, 0.35), 1.35));
  CHECK(profile.nearRange(0.45));
  CHECK(!profile.nearRange(0.451));
  double previous = 0.0;
  for (int sample = 0; sample <= 40; ++sample) {
    const double blend = profile.nearBlend(0.45 - sample * 0.005);
    CHECK(blend >= previous - 1e-12);
    CHECK(blend >= 0.0 && blend <= 1.0);
    previous = blend;
  }
  for (double endpoint : {0.25, 0.45}) {
    CHECK(std::fabs(profile.nearBlend(endpoint + 1e-6) -
                    profile.nearBlend(endpoint - 1e-6)) < 1e-8);
  }
}

void testApproachSpeedAndBurstTaper() {
  Fixture far;
  auto output = far.approach(0.50);
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.5));
  CHECK(far.step(1.99).phase == Phase::APPROACH);
  CHECK(far.step(0.02).phase == Phase::PITCH_CENTER);

  Fixture middle;
  output = middle.approach(0.35);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.35));
  CHECK(middle.step(1.24).phase == Phase::APPROACH);
  CHECK(middle.step(0.02).phase == Phase::PITCH_CENTER);

  Fixture close;
  output = close.approach(0.24);
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.2));
  CHECK(close.step(0.49).phase == Phase::APPROACH);
  output = close.step(0.02);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(output.actuation == Actuation::CENTER_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(!close.controller.allowsYawAlignment(close.input));
  CHECK(close.step(0.01, false).phase == Phase::PITCH_CENTER);
  CHECK(close.step(0.01).phase == Phase::APPROACH);

  // Entering the close range shortens an existing coarse burst immediately.
  Fixture approaching;
  approaching.approach(0.50);
  CHECK(approaching.step(0.60).phase == Phase::APPROACH);
  approaching.input.camera_z_m = 0.24;
  CHECK(approaching.controller.needsApproachUpdate(approaching.input));
  CHECK(approaching.step(0.01).phase == Phase::PITCH_CENTER);
}

void testDistanceTapersPitchLeadAndRate() {
  for (double direction : {-1.0, 1.0}) {
    Fixture far;
    far.imageError(direction * 10.0 * kDeg);
    auto output = far.step();
    CHECK(near(output.pitch_rad - far.input.pitch_rad, direction * 1.5 * kDeg));
    for (int tick = 0; tick < 20; ++tick) output = far.step();
    CHECK(near(output.pitch_rad - far.input.pitch_rad, direction * 6.0 * kDeg));

    Fixture close;
    close.input.camera_z_m = 0.24;
    close.imageError(direction * 10.0 * kDeg);
    output = close.step();
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(near(output.pitch_rad - close.input.pitch_rad, direction * 0.75 * kDeg));
    for (int tick = 0; tick < 20; ++tick) output = close.step();
    CHECK(near(output.pitch_rad - close.input.pitch_rad, direction * 3.0 * kDeg));
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));

    Fixture fine;
    fine.input.camera_z_m = 0.25;
    fine.imageError(direction * 2.5 * kDeg);
    output = fine.step();
    CHECK(near(output.pitch_rad - fine.input.pitch_rad, direction * 0.25 * kDeg));
    for (int tick = 0; tick < 20; ++tick) output = fine.step();
    // One degree of target lead still crosses the physical 0.5 degree deadband.
    CHECK(near(output.pitch_rad - fine.input.pitch_rad, direction * 1.0 * kDeg));
  }

  Fixture middle;
  middle.input.camera_z_m = 0.35;
  middle.imageError(10.0 * kDeg);
  CHECK(near(middle.step().pitch_rad - middle.input.pitch_rad, 1.125 * kDeg));

  // No camera-based taper changes the working initial tag-search rate.
  Fixture search;
  search.input.camera_z_m = 0.24;
  CHECK(near(search.step().pitch_rad - search.input.pitch_rad, 1.5 * kDeg));
}

void testPitchLimitStallCanReachHandoff() {
  Fixture fixture;
  auto output = fixture.atPitchLimit();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
  CHECK(near(output.zipper_motor_velocity_rad_s, 1.2));
  CHECK(near(output.pitch_rad, 69.6 * kDeg));
  CHECK(!fixture.controller.needsPitchCentering(fixture.input));
  CHECK(output.status.find("pitch limit") != std::string::npos);

  output = fixture.step(0.51);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  // The saturation exception never bypasses the post-stop camera capture.
  output = fixture.step(0.01, false);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  output = fixture.step(0.01);
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.pitch_rad, 69.6 * kDeg));

  fixture.input.camera_z_m = 0.205;
  fixture.imageError(4.8 * kDeg);
  CHECK(fixture.controller.needsInsertionUpdate(fixture.input));
  output = fixture.step(0.01);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(output.actuation == Actuation::HOLD_PITCH_YAW);
  CHECK(output.dock_command == Controller::DockCommand::ACTIVATE);
  CHECK(near(output.handoff_camera_z_m, 0.205));
  CHECK(near(output.pitch_rad, 69.6 * kDeg));

  Fixture lower;
  output = lower.atPitchLimit(0.214, 0.4, -4.8);
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.pitch_rad, 0.4 * kDeg));
}

void testPitchLimitApproachUsesMeasuredHoldAndStillStops() {
  Fixture fixture;
  fixture.approach(0.50);  // This burst originally holds 20 degrees.
  fixture.input.pitch_rad = 69.6 * kDeg;
  fixture.input.camera_z_m = 0.214;
  fixture.imageError(4.8 * kDeg);
  CHECK(!fixture.controller.needsPitchCentering(fixture.input));
  auto output = fixture.step();
  CHECK(output.phase == Phase::APPROACH);
  CHECK(near(output.pitch_rad, 69.6 * kDeg));
  fixture.input.pitch_rad = 69.8 * kDeg;
  output = fixture.step();
  CHECK(near(output.pitch_rad, 69.6 * kDeg));  // A fixed hold, not encoder noise following.

  fixture.imageError(6.1 * kDeg);
  CHECK(fixture.controller.needsPitchCentering(fixture.input));
  output = fixture.step(0.01);
  CHECK(output.phase == Phase::PITCH_CENTER);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  CHECK(output.pitch_rad <= fixture.config.pitch_max_rad);

  Fixture changed_direction;
  changed_direction.atPitchLimit();
  changed_direction.imageError(-2.5 * kDeg);
  CHECK(changed_direction.controller.needsPitchCentering(changed_direction.input));
  CHECK(changed_direction.step().phase == Phase::PITCH_CENTER);

  Fixture lost_tag;
  lost_tag.atPitchLimit();
  lost_tag.input.pose_fresh = false;
  output = lost_tag.step();
  CHECK(output.phase == Phase::WAIT_FOR_TAG);
  CHECK(output.actuation == Actuation::STOP);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));

  Fixture extension_limit;
  extension_limit.atPitchLimit();
  extension_limit.input.extension_m = extension_limit.config.extension_max_m;
  CHECK(extension_limit.step().phase == Phase::FAILED);
}

void testPitchLimitExceptionRequiresSmallCloseBlockedCorrection() {
  for (int rejected_case = 0; rejected_case < 4; ++rejected_case) {
    Fixture fixture;
    const double depth = rejected_case == 0 ? 0.251 : 0.214;
    const double pitch = rejected_case == 1 ? 68.9 : 69.6;
    const double error = rejected_case == 2 ? -4.8 : rejected_case == 3 ? 6.1 : 4.8;
    const auto output = fixture.atPitchLimit(depth, pitch, error);
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(output.actuation == Actuation::CENTER_PITCH);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
    CHECK(output.pitch_rad >= fixture.config.pitch_min_rad);
    CHECK(output.pitch_rad <= fixture.config.pitch_max_rad);
  }

  // Handoff still outranks pitch centering, even beyond the saturation window.
  Fixture handoff;
  const auto output = handoff.atPitchLimit(0.205, 40.0, 12.0);
  CHECK(output.phase == Phase::DOCK_ACTIVATE);
  CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
}

void testModestPitchErrorDoesNotRepeatLoggedStall() {
  for (double ceiling_deg : {63.0, 70.0}) {
    Controller::Config config;
    config.pitch_max_rad = ceiling_deg * kDeg;
    for (double image_error_deg : {1.3, 1.98}) {
      Fixture fixture(config);
      CHECK(near(fixture.config.centering_start_tolerance_rad, 2.0 * kDeg));
      CHECK(near(fixture.config.centering_stop_tolerance_rad, 4.0 * kDeg));
      // Reproduce the stopped log beyond the 25 cm pitch-limit exception.
      // Ordinary centering acceptance must now permit the next short burst.
      auto output = fixture.atPitchLimit(0.271, 62.58, image_error_deg);
      CHECK(output.phase == Phase::APPROACH);
      CHECK(output.actuation == Actuation::EXTEND_HOLD_PITCH);
      CHECK(near(output.pitch_rad, 62.58 * kDeg));
      CHECK(output.zipper_motor_velocity_rad_s > 0.0);
      CHECK(output.status.find("pitch limit") == std::string::npos);
      CHECK(!fixture.controller.needsPitchCentering(fixture.input));

      // Modest camera drift during a burst keeps its captured measured pitch.
      fixture.input.pitch_rad -= 0.1 * kDeg;
      fixture.imageError(3.5 * kDeg);
      output = fixture.step();
      CHECK(output.phase == Phase::APPROACH);
      CHECK(near(output.pitch_rad, 62.58 * kDeg));

      // Retain the 4 degree drift stop, then resume once back below 2 degrees.
      fixture.imageError(4.1 * kDeg);
      CHECK(fixture.controller.needsPitchCentering(fixture.input));
      output = fixture.step();
      CHECK(output.phase == Phase::PITCH_CENTER);
      CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
      fixture.imageError(1.98 * kDeg);
      output = fixture.step();
      CHECK(output.phase == Phase::APPROACH);
      CHECK(near(output.pitch_rad, fixture.input.pitch_rad));
      CHECK(output.zipper_motor_velocity_rad_s > 0.0);
    }

    Fixture outside_window(config);
    const auto output = outside_window.atPitchLimit(0.271, 62.58, 2.02);
    CHECK(output.phase == Phase::PITCH_CENTER);
    CHECK(output.actuation == Actuation::CENTER_PITCH);
    CHECK(near(output.zipper_motor_velocity_rad_s, 0.0));
  }
}

void testNearApproachConfigValidation() {
  using Field = double Controller::Config::*;
  const Field positive_fields[] = {
    &Controller::Config::near_zipper_motor_velocity_rad_s,
    &Controller::Config::near_approach_burst_duration_sec,
    &Controller::Config::near_pitch_scale,
    &Controller::Config::pitch_limit_approach_window_m,
    &Controller::Config::pitch_limit_margin_rad,
    &Controller::Config::pitch_limit_max_vertical_error_rad
  };
  for (Field field : positive_fields) {
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()}) {
      Controller::Config config;
      config.*field = invalid;
      Fixture fixture(config);
      CHECK(fixture.controller.phase() == Phase::FAILED);
    }
  }
  for (int invalid_case = 0; invalid_case < 10; ++invalid_case) {
    Controller::Config config;
    if (invalid_case == 0) config.near_zipper_motor_velocity_rad_s = 1.19;
    if (invalid_case == 1) config.near_zipper_motor_velocity_rad_s = 1.51;
    if (invalid_case == 2) config.near_approach_burst_duration_sec = 2.01;
    if (invalid_case == 3) config.near_pitch_scale = 1.01;
    if (invalid_case == 4) config.pitch_limit_approach_window_m = 0.205;
    if (invalid_case == 5) config.pitch_limit_margin_rad = config.pitch_max_rad - config.pitch_min_rad;
    if (invalid_case == 6) config.pitch_limit_max_vertical_error_rad = 1.0 * kDeg;
    if (invalid_case == 7) config.pitch_limit_max_vertical_error_rad = 90.0 * kDeg;
    if (invalid_case == 8) config.approach_profile.slowdown_end_m = 0.45;
    if (invalid_case == 9) config.approach_profile.slowdown_start_m =
        std::numeric_limits<double>::quiet_NaN();
    Fixture fixture(config);
    CHECK(fixture.controller.phase() == Phase::FAILED);
  }
}
}  // namespace

int main() {
  try {
    testDistanceProfile();
    testApproachSpeedAndBurstTaper();
    testDistanceTapersPitchLeadAndRate();
    testPitchLimitStallCanReachHandoff();
    testPitchLimitApproachUsesMeasuredHoldAndStillStops();
    testPitchLimitExceptionRequiresSmallCloseBlockedCorrection();
    testModestPitchErrorDoesNotRepeatLoggedStall();
    testNearApproachConfigValidation();
    std::cout << "Near docking approach tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Near docking approach test failed: " << error.what() << '\n';
    return 1;
  }
}
