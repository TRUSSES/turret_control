#include <gtest/gtest.h>
#include <pigpio.h>
#include "limit_switch.h"
#include "config.h"
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>

// Test fixture that initializes pigpio and loads the configuration file.
class LimitSwitchTest : public ::testing::Test {
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

TEST_F(LimitSwitchTest, SpiralZipperSwitchTest) {
  const auto& config = Config::Instance().GetConfig();
  // Retrieve the spiral zipper limit switch pin from the configuration.
  int spiral_pin = config["spiral_zipper"]["limit_switch_pin"].as<int>();
  int debounce_ms = config["spiral_zipper"]["debounce_threshold_ms"].as<int>();

  // Instantiate a LimitSwitch from the configuration.
  LimitSwitch spiralSwitch(spiral_pin, debounce_ms);

  std::atomic<int> pressCount{0};
  std::atomic<int> releaseCount{0};

  spiralSwitch.SetStateChangeCallback([&](bool state) {
    if (state) {
      pressCount++;
      std::cout << "[Spiral] Pressed (" << pressCount.load() << ")\n";
    } else {
      releaseCount++;
      std::cout << "[Spiral] Released (" << releaseCount.load() << ")\n";
    }
  });

  std::cout << "\n[Spiral Zipper Test] Please press and release the SPIRAL ZIPPER limit switch 3 times.\n";

  auto start = std::chrono::steady_clock::now();
  while ((pressCount < 3 || releaseCount < 3) &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[Spiral Zipper Test] Final: Presses: " << pressCount.load()
            << ", Releases: " << releaseCount.load() << "\n";
  
  EXPECT_EQ(pressCount.load(), 3) << "Expected 3 press events for spiral switch.";
  EXPECT_EQ(releaseCount.load(), 3) << "Expected 3 release events for spiral switch.";
}

TEST_F(LimitSwitchTest, TurretSwitchTest) {
  const auto& config = Config::Instance().GetConfig();
  // Retrieve the turret limit switch pin from the configuration.
  int turret_pin = config["turret_limit_switch"]["limit_switch_pin"].as<int>();
  int debounce_ms = config["turret_limit_switch"]["debounce_threshold_ms"].as<int>();

  LimitSwitch turretSwitch(turret_pin, debounce_ms);

  std::atomic<int> pressCount{0};
  std::atomic<int> releaseCount{0};

  turretSwitch.SetStateChangeCallback([&](bool state) {
    if (state) {
      pressCount++;
      std::cout << "[Turret] Pressed (" << pressCount.load() << ")\n";
    } else {
      releaseCount++;
      std::cout << "[Turret] Released (" << releaseCount.load() << ")\n";
    }
  });

  std::cout << "\n[Turret Test] Please press and release the TURRET limit switch 3 times.\n";

  auto start = std::chrono::steady_clock::now();
  while ((pressCount < 3 || releaseCount < 3) &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[Turret Test] Final: Presses: " << pressCount.load()
            << ", Releases: " << releaseCount.load() << "\n";
  
  EXPECT_EQ(pressCount.load(), 3) << "Expected 3 press events for turret switch.";
  EXPECT_EQ(releaseCount.load(), 3) << "Expected 3 release events for turret switch.";
}
