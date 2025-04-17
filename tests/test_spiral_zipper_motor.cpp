#include <gtest/gtest.h>
#include <pigpio.h>
#include "servo_city_motor.h"
#include "config.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

// Define test configuration constants.
namespace TestConfig {
  // Desired target velocity (absolute value) for both forward and reverse tests.
  // The forward target will be positive, and the reverse target is the negative of this value.
  const double kTargetVelocity = 2.0;  // rad/s
  
  // Tolerance for the velocity measurement (allowed deviation from target).
  const double kVelocityTolerance = 0.1;  // rad/s
  
  // Time allowed for the motor to settle after setting a new target.
  const auto kSettleTime = std::chrono::seconds(1);
  
  // Duration over which the test measures the velocity.
  const auto kMeasurementTime = std::chrono::seconds(5);
  
  // Delay between each control loop update during the test.
  const auto kUpdateDelay = std::chrono::milliseconds(50);
}

// Test fixture for ServoCityMotor tests. Initializes pigpio and loads config.
class ServoCityMotorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int ret = gpioInitialise();
    ASSERT_GE(ret, 0) << "Failed to initialize pigpio.";
    // Adjust the path to your config file if needed.
    ASSERT_TRUE(Config::Instance().Load("../config/config.yaml")) << "Failed to load config.yaml";
  }
  
  static void TearDownTestSuite() {
    gpioTerminate();
  }
};

// Test case: Verify the motor reaches its target velocity within the allotted time.
TEST_F(ServoCityMotorTest, TargetVelocityReachedTest) {
  YAML::Node config = Config::Instance().GetConfig();
  // Here, we're using the same YAML node for this test as well.
  // (Assuming that for testing purposes, a node called "servo_city_motor" exists.)
  ServoCityMotor motor(config["spiral_zipper"]);
  
  // Forward test: set target to positive kTargetVelocity.
  double targetForward = TestConfig::kTargetVelocity;
  motor.setTargetVelocity(targetForward);
  std::cout << "\n[Target Velocity Reached Test - Forward] Waiting for target " 
            << targetForward << " rad/s..." << std::endl;
  
  bool reachedForward = false;
  auto start = std::chrono::steady_clock::now();
  // Wait for up to 5 seconds for the motor to reach the target.
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    motor.update();
    double current = motor.getCurrentVelocity();
    std::cout << "Current velocity: " << current << " rad/s\r" << std::flush;
    if (std::fabs(current - targetForward) < TestConfig::kVelocityTolerance) {
      reachedForward = true;
      break;
    }
    std::this_thread::sleep_for(TestConfig::kUpdateDelay);
  }
  std::cout << "\n[Forward] Target reached: " << reachedForward << std::endl;
  EXPECT_TRUE(reachedForward) << "Motor did not reach forward target velocity within timeout.";

  // Stop between tests.
  motor.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Reverse test: set target to negative kTargetVelocity.
  double targetBackward = -TestConfig::kTargetVelocity;
  motor.setTargetVelocity(targetBackward);
  std::cout << "\n[Target Velocity Reached Test - Reverse] Waiting for target " 
            << targetBackward << " rad/s..." << std::endl;
  
  bool reachedBackward = false;
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    motor.update();
    double current = motor.getCurrentVelocity();
    std::cout << "Current velocity: " << current << " rad/s\r" << std::flush;
    if (std::fabs(current - targetBackward) < TestConfig::kVelocityTolerance) {
      reachedBackward = true;
      break;
    }
    std::this_thread::sleep_for(TestConfig::kUpdateDelay);
  }
  std::cout << "\n[Reverse] Target reached: " << reachedBackward << std::endl;
  EXPECT_TRUE(reachedBackward) << "Motor did not reach reverse target velocity within timeout.";

  motor.stop();
}

// Test case: Verify that the motor maintains its target velocity throughout a 5-second run.
TEST_F(ServoCityMotorTest, MaintainVelocityTest) {
  YAML::Node config = Config::Instance().GetConfig();
  ServoCityMotor motor(config["spiral_zipper"]);
  
  auto checkMaintainedVelocity = [&](double target) -> bool {
    motor.setTargetVelocity(target);
    // Settling period to allow the motor to stabilize.
    auto settleStart = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - settleStart < TestConfig::kSettleTime) {
      motor.update();
      std::this_thread::sleep_for(TestConfig::kUpdateDelay);
    }
  
    bool maintained = true;
    auto sampleStart = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - sampleStart < TestConfig::kMeasurementTime) {
      motor.update();
      double currentVel = motor.getCurrentVelocity();
      std::cout << (target > 0 ? "Forward" : "Reverse")
                << ": current velocity = " << currentVel << " rad/s\r" << std::flush;
      if (std::fabs(currentVel - target) > TestConfig::kVelocityTolerance) {
        maintained = false;
        std::cout << "\nVelocity out of tolerance: " << currentVel << " rad/s" << std::endl;
      }
      std::this_thread::sleep_for(TestConfig::kUpdateDelay);
    }
    return maintained;
  };

  // Check forward maintenance.
  double targetForward = TestConfig::kTargetVelocity;
  bool forwardMaintained = checkMaintainedVelocity(targetForward);
  std::cout << "\n[Forward Maintain Test] Velocity maintained: " 
            << (forwardMaintained ? "Yes" : "No") << std::endl;
  EXPECT_TRUE(forwardMaintained) << "Motor did not maintain forward target velocity.";
  
  motor.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check reverse maintenance.
  double targetBackward = -TestConfig::kTargetVelocity;
  bool backwardMaintained = checkMaintainedVelocity(targetBackward);
  std::cout << "\n[Reverse Maintain Test] Velocity maintained: " 
            << (backwardMaintained ? "Yes" : "No") << std::endl;
  EXPECT_TRUE(backwardMaintained) << "Motor did not maintain reverse target velocity.";
  
  motor.stop();
}
