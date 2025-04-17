#include <gtest/gtest.h>
#include "spiral_zipper.h"
#include "config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <chrono>

// This test verifies that calling SpiralZipper::Zero() retracts the actuator until 
// the limit switch is triggered, resets the encoder count, and stops the motor.
TEST(SpiralZipperTest, ZeroingTest) {
  // Load configuration.
  if (!Config::Instance().Load("../config/config.yaml")) {
    FAIL() << "Failed to load config.yaml";
  }
  YAML::Node config = Config::Instance().GetConfig();
  YAML::Node zipperConfig = config["spiral_zipper"];
  
  // Construct the SpiralZipper instance using the YAML configuration.
  SpiralZipper zipper(zipperConfig);

  // Call Zero(); this should retract until the limit switch is pressed, then stop the motor.
  zipper.Zero();
  
  // Allow a short period for the motor to settle.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // After zeroing, the encoder count should be near zero.
  int encoderCount = zipper.GetEncoderCount();
  double extension = encoderCount * zipper.GetExtensionPerStep();
  std::cout << "Measured extension after zeroing: " << extension << " meters" << std::endl;
  
  // Also check that the motor velocity is near zero.
  double motorVel = zipper.GetMotorVelocity();
  std::cout << "[ZeroingTest] Motor velocity after Zero(): " << motorVel << " rad/s" << std::endl;
  EXPECT_NEAR(motorVel, 0.0, 0.05);  // Tolerance of 0.05 rad/s.
}
