#include <gtest/gtest.h>
#include "limit_switch.h"
#include "config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

// Test case for the Spiral Zipper Limit Switch using the shared config.
TEST(LimitSwitchTest, SpiralZipperSwitchTriggeredFromConfig) {
  // Load configuration.
  if (!Config::Instance().Load("config.yaml")) {
    FAIL() << "Failed to load config.yaml";
  }
  YAML::Node config = Config::Instance().GetConfig();
  YAML::Node spiralConfig = config["spiral_zipper"];
  
  // Construct the limit switch using values from the config.
  LimitSwitch spiralLimit(spiralConfig);
  std::atomic<bool> spiralTriggered(false);
  
  spiralLimit.SetStateChangeCallback([&spiralTriggered](bool pressed) {
    if (pressed) {
      spiralTriggered = true;
      std::cout << "Spiral Zipper Limit Switch triggered (from config)." << std::endl;
    }
  });
  
  std::cout << "\n----- Test Spiral Zipper Limit Switch (Config) -----\n";
  std::cout << "Please trigger the Spiral Zipper Limit Switch on GPIO "
            << spiralConfig["limit_switch_pin"].as<int>() << std::endl;
  std::cout << "Then press Enter to continue..." << std::endl;
  std::cin.get();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  EXPECT_TRUE(spiralTriggered.load());
}

// Test case for the Turret Limit Switch using the shared config.
TEST(LimitSwitchTest, TurretSwitchTriggeredFromConfig) {
  // Load configuration.
  if (!Config::Instance().Load("config/config.yaml")) {
    FAIL() << "Failed to load config.yaml";
  }
  YAML::Node config = Config::Instance().GetConfig();
  YAML::Node turretConfig = config["turret_limit_switch"];
  
  // Construct the turret limit switch object using YAML values.
  LimitSwitch turretLimit(turretConfig);
  std::atomic<bool> turretTriggered(false);
  
  turretLimit.SetStateChangeCallback([&turretTriggered](bool pressed) {
    if (pressed) {
      turretTriggered = true;
      std::cout << "Turret Limit Switch triggered (from config)." << std::endl;
    }
  });
  
  std::cout << "\n----- Test Turret Limit Switch (Config) -----\n";
  std::cout << "Please trigger the Turret Limit Switch on GPIO "
            << turretConfig["limit_switch_pin"].as<int>() << std::endl;
  std::cout << "Then press Enter to continue..." << std::endl;
  std::cin.get();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  EXPECT_TRUE(turretTriggered.load());
}
