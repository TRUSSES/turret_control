#include <gtest/gtest.h>
#include "spiral_zipper.h"
#include "config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <chrono>

// This test verifies that calling SpiralZipper::Zero() retracts the mechanism until 
// the limit switch is triggered and then resets the encoder count to near zero.
TEST(SpiralZipperTest, ZeroingWorksFromConfig) {
  // Load configuration.
  if (!Config::Instance().Load("config/config.yaml")) {
    FAIL() << "Failed to load config.yaml";
  }
  YAML::Node config = Config::Instance().GetConfig();
  YAML::Node zipperConfig = config["spiral_zipper"];
  
  // Construct the SpiralZipper instance using the YAML configuration.
  SpiralZipper zipper(zipperConfig);
  
  std::cout << "\n----- Spiral Zipper Zeroing Test (Config) -----\n";
  std::cout << "Ensure the spiral zipper is extended and its limit switch is not triggered." << std::endl;
  std::cout << "When the test begins, the motor will start retracting slowly." << std::endl;
  std::cout << "Please manually trigger the spiral zipper limit switch on GPIO "
            << zipperConfig["limit_switch_pin"].as<int>()
            << " to halt retraction, then press Enter..." << std::endl;
  
  std::cin.get();
  
  // Call Zero(), which should retract until the limit switch triggers.
  zipper.Zero();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // After zeroing, the encoder count should be approximately zero.
  int encoderCount = zipper.GetEncoderCount();
  double extension = encoderCount * zipper.GetExtensionPerStep();
  std::cout << "Measured extension after zeroing: " << extension << " meters" << std::endl;
  
  EXPECT_NEAR(extension, 0.0, 1e-6);
}
