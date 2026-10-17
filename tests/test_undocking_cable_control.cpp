#include "undocking_cable_control.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Controller = turret_control::UndockingCableController;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    throw std::runtime_error("line " + std::to_string(line) + ": " + expression);
  }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(double left, double right, double tolerance = 1e-9) {
  return std::fabs(left - right) <= tolerance;
}

double command(Controller& controller, double now, double extension,
               double feedback = 0.0, bool retracting = true,
               bool lower_limit = false, double cap = 1.2) {
  return controller.computeVelocity(now, extension, 0.9, 0.013,
                                     feedback, cap, retracting, lower_limit);
}

void testMeasuredRetractionDrivesTakeupAtZeroPitchError() {
  Controller controller;
  // No previous velocity can be assumed on the first stationary tick.
  CHECK(near(command(controller, 0.0, 0.5), 0.0));
  // Travel 0.1 mm in 25 ms: -0.004 m/s * 0.9 / 0.013 m.
  const double velocity = command(controller, 0.025, 0.4999);
  CHECK(near(velocity, -0.004 * 0.9 / 0.013));
  CHECK(velocity < -0.25);
}

void testRateWindowUsesTravelAcrossSeveralSamples() {
  Controller controller;
  CHECK(near(command(controller, 0.0, 0.5), 0.0));
  CHECK(near(command(controller, 0.005, 0.49998), 0.0));
  CHECK(near(command(controller, 0.010, 0.49996), 0.0));
  CHECK(near(command(controller, 0.015, 0.49994), 0.0));
  const double measured = command(controller, 0.025, 0.4999);
  CHECK(near(measured, -0.004 * 0.9 / 0.013));
  // Between complete measurement windows, preserve the last measured rate.
  CHECK(near(command(controller, 0.030, 0.49988), measured));
  CHECK(near(command(controller, 0.050, 0.4998), measured));
}

void testStoppedZipperAndRestartCannotReuseOldMotion() {
  Controller controller;
  command(controller, 0.0, 0.5);
  CHECK(command(controller, 0.025, 0.4999) < 0.0);
  // A floor/limit/port hold disables feedforward immediately, but retains
  // needed pitch feedback, including correction while the zipper is stopped.
  CHECK(near(command(controller, 0.030, 0.49988, -0.20, false), -0.20));
  CHECK(near(command(controller, 0.035, 0.49988), 0.0));
  CHECK(near(command(controller, 0.060, 0.49988), 0.0));
  CHECK(command(controller, 0.085, 0.49978) < 0.0);
  controller.reset();
  CHECK(near(command(controller, 0.090, 0.49976), 0.0));
}

void testMeasuredStandstillAndPositiveTravelDoNotWindCable() {
  Controller controller;
  command(controller, 0.0, 0.5);
  CHECK(command(controller, 0.025, 0.4999) < 0.0);
  CHECK(near(command(controller, 0.050, 0.4999), 0.0));
  CHECK(near(command(controller, 0.075, 0.5000), 0.0));
}

void testFeedbackBlendsWithTakeupAndTotalRespectsMotorCap() {
  Controller controller;
  command(controller, 0.0, 0.5);
  const double feedforward = -0.004 * 0.9 / 0.013;
  // Positive feedback can oppose takeup to correct the measured joint angle.
  CHECK(near(command(controller, 0.025, 0.4999, 0.20), feedforward + 0.20));
  CHECK(near(command(controller, 0.050, 0.4998, 0.50), feedforward + 0.50));
  CHECK(near(command(controller, 0.075, 0.4997, -1.20), -1.20));
  CHECK(near(command(controller, 0.100, 0.4996, -0.20, true, false, 0.25), -0.25));
  // Even a very large measured travel sample remains bounded by the caller.
  CHECK(near(command(controller, 0.125, 0.4, 0.0), -1.20));
}

void testLowerPitchSwitchIsAppliedAfterFeedforward() {
  Controller controller;
  command(controller, 0.0, 0.5);
  CHECK(near(command(controller, 0.025, 0.4999, 0.0, true, true), 0.0));
  CHECK(near(command(controller, 0.050, 0.4998, 0.20, true, true), 0.0));
  // The switch still permits motion away from it after both terms are added.
  const double expected = 0.50 - 0.004 * 0.9 / 0.013;
  CHECK(near(command(controller, 0.075, 0.4997, 0.50, true, true), expected));
}

void testGeometryAndSpoolRadiusDetermineUnitsAndSign() {
  Controller controller;
  controller.computeVelocity(0.0, 0.5, 0.5, 0.020, 0.0, 1.2, true);
  CHECK(near(controller.computeVelocity(0.025, 0.4999, 0.5, 0.020,
                                       0.0, 1.2, true), -0.10));
  CHECK(near(controller.computeVelocity(0.050, 0.4998, -0.5, 0.020,
                                       0.0, 1.2, true), 0.10));
}

void testDelayedOrNonMonotonicSamplesEstablishNewBaseline() {
  Controller controller;
  command(controller, 0.0, 0.5);
  CHECK(command(controller, 0.025, 0.4999) < 0.0);
  CHECK(near(command(controller, 0.2, 0.4990), 0.0));
  CHECK(near(command(controller, 0.225, 0.4990), 0.0));
  CHECK(command(controller, 0.250, 0.4989) < 0.0);
  CHECK(near(command(controller, 0.240, 0.4989), 0.0));
  CHECK(near(command(controller, 0.265, 0.4989), 0.0));
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

void testInvalidInputsClearTravelHistory() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  Controller controller;
  for (double invalid : {nan, infinity}) {
    rejectsInvalidArgument([&] {
      controller.computeVelocity(invalid, 0.5, 0.9, 0.013, 0.0, 1.2, true);
    });
    rejectsInvalidArgument([&] {
      controller.computeVelocity(0.0, invalid, 0.9, 0.013, 0.0, 1.2, true);
    });
    rejectsInvalidArgument([&] {
      controller.computeVelocity(0.0, 0.5, invalid, 0.013, 0.0, 1.2, true);
    });
    rejectsInvalidArgument([&] {
      controller.computeVelocity(0.0, 0.5, 0.9, 0.013, invalid, 1.2, true);
    });
  }
  for (double radius : {0.0, -0.013, nan, infinity}) {
    rejectsInvalidArgument([&] {
      controller.computeVelocity(0.0, 0.5, 0.9, radius, 0.0, 1.2, true);
    });
  }
  for (double cap : {0.0, 0.19, 2.01, nan, infinity}) {
    rejectsInvalidArgument([&] {
      controller.computeVelocity(0.0, 0.5, 0.9, 0.013, 0.0, cap, true);
    });
  }
  command(controller, 0.0, 0.5);
  CHECK(command(controller, 0.025, 0.4999) < 0.0);
  rejectsInvalidArgument([&] { command(controller, nan, 0.4998); });
  CHECK(near(command(controller, 0.030, 0.4998), 0.0));
}
}  // namespace

int main() {
  try {
    testMeasuredRetractionDrivesTakeupAtZeroPitchError();
    testRateWindowUsesTravelAcrossSeveralSamples();
    testStoppedZipperAndRestartCannotReuseOldMotion();
    testMeasuredStandstillAndPositiveTravelDoNotWindCable();
    testFeedbackBlendsWithTakeupAndTotalRespectsMotorCap();
    testLowerPitchSwitchIsAppliedAfterFeedforward();
    testGeometryAndSpoolRadiusDetermineUnitsAndSign();
    testDelayedOrNonMonotonicSamplesEstablishNewBaseline();
    testInvalidInputsClearTravelHistory();
    std::cout << "Undocking cable control tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
