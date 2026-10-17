#include "docking_pitch_control.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::DockingPitchController;
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

template <typename Callback>
void rejectsInvalidArgument(Callback callback) {
  bool rejected = false;
  try {
    callback();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  CHECK(rejected);
}

void testSmallErrorsHoldAndFineCorrectionsRemainSlow() {
  Controller controller;
  for (double degrees : {0.0, 0.1, 0.5}) {
    CHECK(near(controller.computeVelocity(degrees * kDeg, 0.8), 0.0));
    CHECK(near(controller.computeVelocity(-degrees * kDeg, 0.8), 0.0));
  }
  const double just_outside = controller.computeVelocity(0.501 * kDeg, 0.8);
  CHECK(just_outside >= 0.20);
  CHECK(just_outside < 0.201);
  const double fine = controller.computeVelocity(1.0 * kDeg, 0.8);
  CHECK(fine > 0.20);
  CHECK(fine < 0.25);
  CHECK(near(controller.computeVelocity(2.0 * kDeg, 0.8), 0.25));
}

void testLargeCorrectionsUseCoarseSpeedInBothDirections() {
  Controller controller;
  const double transition = controller.computeVelocity(4.0 * kDeg, 0.8);
  CHECK(transition > 0.25);
  CHECK(transition < 0.8);
  for (double degrees : {6.0, 20.0, 40.0}) {
    CHECK(near(controller.computeVelocity(degrees * kDeg, 0.8), 0.8));
    CHECK(near(controller.computeVelocity(-degrees * kDeg, 0.8), -0.8));
  }
  // At this error the previous proportional controller produced only 0.25.
  CHECK(controller.computeVelocity(6.0 * kDeg, 0.8) > 0.7);
}

void testCallerCapsAreRespectedEvenBelowConfiguredFineSpeed() {
  Controller controller;
  for (double cap : {0.20, 0.22, 0.35, 0.8, 2.0}) {
    for (double degrees : {0.51, 1.0, 2.0, 4.0, 6.0, 40.0}) {
      const double positive = controller.computeVelocity(degrees * kDeg, cap);
      const double negative = controller.computeVelocity(-degrees * kDeg, cap);
      CHECK(positive >= 0.20);
      CHECK(positive <= cap + 1e-9);
      CHECK(near(negative, -positive));
    }
    CHECK(near(controller.computeVelocity(40.0 * kDeg, cap), cap));
  }
  CHECK(near(controller.computeVelocity(2.0 * kDeg, 0.22), 0.22));
}

void testLowerSwitchOnlyBlocksMotionTowardThatSwitch() {
  Controller controller;
  for (double degrees : {0.51, 1.0, 4.0, 40.0}) {
    CHECK(near(controller.computeVelocity(-degrees * kDeg, 0.8, true), 0.0));
    CHECK(near(controller.computeVelocity(degrees * kDeg, 0.8, true),
               controller.computeVelocity(degrees * kDeg, 0.8, false)));
    CHECK(controller.computeVelocity(-degrees * kDeg, 0.8, false) < 0.0);
  }
}

void testSpeedIsContinuousAndMonotonicAcrossProfileTransitions() {
  Controller controller;
  for (double boundary : {2.0 * kDeg, 6.0 * kDeg}) {
    const double below = controller.computeVelocity(boundary - 1e-7, 0.8);
    const double above = controller.computeVelocity(boundary + 1e-7, 0.8);
    CHECK(above >= below);
    CHECK(std::fabs(above - below) < 1e-8);
  }
  double previous = 0.20;
  for (int hundredths = 51; hundredths <= 1000; ++hundredths) {
    const double speed = controller.computeVelocity(hundredths * 0.01 * kDeg, 0.8);
    CHECK(speed >= previous - 1e-12);
    CHECK(speed <= 0.8 + 1e-12);
    previous = speed;
  }
}

void testCustomProfileChangesEndpoints() {
  Controller::Config config;
  config.fine_velocity_rad_s = 0.30;
  config.fine_error_rad = 3.0 * kDeg;
  config.coarse_error_rad = 9.0 * kDeg;
  CHECK(Controller::validConfig(config));
  Controller controller(config);
  CHECK(near(controller.computeVelocity(3.0 * kDeg, 0.75), 0.30));
  CHECK(near(controller.computeVelocity(9.0 * kDeg, 0.75), 0.75));
  CHECK(near(controller.computeVelocity(-40.0 * kDeg, 0.75), -0.75));
}

void testInvalidProfilesAndInputsAreRejected() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  auto invalidProfile = [](Controller::Config config) {
    CHECK(!Controller::validConfig(config));
    rejectsInvalidArgument([&] { Controller rejected(config); });
  };
  for (double speed : {0.19, 2.01, nan, infinity}) {
    Controller::Config config;
    config.fine_velocity_rad_s = speed;
    invalidProfile(config);
  }
  for (double fine_error : {0.0, 0.5 * kDeg, nan, infinity}) {
    Controller::Config config;
    config.fine_error_rad = fine_error;
    invalidProfile(config);
  }
  for (double coarse_error : {0.0, 1.0 * kDeg, 2.0 * kDeg, nan, infinity}) {
    Controller::Config config;
    config.coarse_error_rad = coarse_error;
    invalidProfile(config);
  }
  Controller controller;
  for (double error : {nan, infinity, -infinity}) {
    rejectsInvalidArgument([&] { controller.computeVelocity(error, 0.8); });
  }
  for (double cap : {0.0, -0.2, 0.19, 2.01, nan, infinity}) {
    rejectsInvalidArgument([&] { controller.computeVelocity(4.0 * kDeg, cap); });
  }
}
}  // namespace

int main() {
  try {
    testSmallErrorsHoldAndFineCorrectionsRemainSlow();
    testLargeCorrectionsUseCoarseSpeedInBothDirections();
    testCallerCapsAreRespectedEvenBelowConfiguredFineSpeed();
    testLowerSwitchOnlyBlocksMotionTowardThatSwitch();
    testSpeedIsContinuousAndMonotonicAcrossProfileTransitions();
    testCustomProfileChangesEndpoints();
    testInvalidProfilesAndInputsAreRejected();
    std::cout << "Docking pitch profile tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
