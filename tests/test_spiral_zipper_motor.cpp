#include <gtest/gtest.h>
#include <pigpio.h>
#include "servo_city_motor.h"
#include "config.h"
#include <chrono>
#include <thread>
#include <iostream>

// Test fixture for ServoCityMotor tests.
class ServoCityMotorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int ret = gpioInitialise();
    ASSERT_GE(ret, 0) << "Failed to initialize pigpio.";
    // Load configuration, adjust path if necessary.
    ASSERT_TRUE(Config::Instance().Load("../config/config.yaml")) << "Failed to load config.yaml";
  }
  
  static void TearDownTestSuite() {
    gpioTerminate();
  }
};

TEST_F(ServoCityMotorTest, BasicMovementTest) {
  YAML::Node config = Config::Instance().GetConfig();
  // Construct the ServoCityMotor using the YAML node "servo_city_motor"
  ServoCityMotor motor(config["spiral_zipper"]);

  // First, test forward movement
  double targetVelocityForward = 0.1;  // rad/s
  motor.setTargetVelocity(targetVelocityForward);

  // Settling period: allow the motor to reach steady-state (1 second)
  auto settleStart = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - settleStart < std::chrono::seconds(1)) {
    motor.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  // Measure velocity over 2 seconds.
  double sumForward = 0;
  int samplesForward = 0;
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    motor.update();
    sumForward += motor.getCurrentVelocity();
    samplesForward++;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  double measuredForward = (samplesForward > 0 ? sumForward / samplesForward : 0);
  std::cout << "Measured forward velocity: " << measuredForward << " rad/s" << std::endl;
  EXPECT_GT(measuredForward, 0.05) << "Motor did not move forward as expected.";

  // Stop the motor briefly between tests.
  motor.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Now test backward movement.
  double targetVelocityBackward = -0.1;  // rad/s
  motor.setTargetVelocity(targetVelocityBackward);
  
  // Settling period for backward movement (1 second).
  settleStart = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - settleStart < std::chrono::seconds(1)) {
    motor.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  double sumBackward = 0;
  int samplesBackward = 0;
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    motor.update();
    sumBackward += motor.getCurrentVelocity();
    samplesBackward++;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  double measuredBackward = (samplesBackward > 0 ? sumBackward / samplesBackward : 0);
  std::cout << "Measured backward velocity: " << measuredBackward << " rad/s" << std::endl;
  EXPECT_LT(measuredBackward, -0.05) << "Motor did not move backward as expected.";

  motor.stop();
}

TEST_F(ServoCityMotorTest, PreciseVelocityMeasurementTest) {
  YAML::Node config = Config::Instance().GetConfig();
  ServoCityMotor motor(config["servo_city_motor"]);

  auto measure_velocity = [&](double target, int duration_sec) -> double {
    motor.setTargetVelocity(target);
    
    // Settling period: allow the motor to stabilize (1 second)
    auto settleStart = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - settleStart < std::chrono::seconds(1)) {
      motor.update();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    double sum = 0;
    int samples = 0;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(duration_sec)) {
      motor.update();
      sum += motor.getCurrentVelocity();
      samples++;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Only stop after measurement
    motor.stop();
    return (samples > 0 ? sum / samples : 0);
  };

  double targetForward = 0.1; // rad/s
  double measuredForward = measure_velocity(targetForward, 5);
  std::cout << "Average measured forward velocity: " << measuredForward << " rad/s" << std::endl;
  EXPECT_NEAR(measuredForward, targetForward, 0.02) << "Forward velocity mismatch.";

  double targetBackward = -0.1; // rad/s
  double measuredBackward = measure_velocity(targetBackward, 5);
  std::cout << "Average measured backward velocity: " << measuredBackward << " rad/s" << std::endl;
  EXPECT_NEAR(measuredBackward, targetBackward, 0.02) << "Backward velocity mismatch.";
}
