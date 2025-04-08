#include <gtest/gtest.h>
#include "limit_switch.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

// Test case for the Spiral Zipper Limit Switch.
TEST(LimitSwitchTest, SpiralZipperSwitchTriggered) {
  // Define the GPIO pin for the spiral zipper limit switch.
  int spiralPin = 16;
  
  // Use an atomic flag to track callback invocation.
  std::atomic<bool> spiralTriggered(false);
  
  // Create a LimitSwitch instance with a 30 ms debounce threshold.
  LimitSwitch spiralLimit(spiralPin, 30);
  
  // Set the callback to update the flag when the switch is pressed.
  spiralLimit.SetStateChangeCallback([&spiralTriggered](bool pressed) {
    if (pressed) {
      spiralTriggered = true;
      std::cout << "Spiral Zipper Limit Switch triggered." << std::endl;
    }
  });

  std::cout << "\n----- Test Spiral Zipper Limit Switch -----\n";
  std::cout << "Please trigger the Spiral Zipper Limit Switch on GPIO " << spiralPin << std::endl;
  std::cout << "Then press Enter to continue..." << std::endl;
  
  // Wait for the user to press Enter.
  std::cin.get();

  // Allow time for asynchronous callbacks.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Verify that the spiral limit switch was triggered.
  EXPECT_TRUE(spiralTriggered.load());
}

// Test case for the Turret Limit Switch.
TEST(LimitSwitchTest, TurretSwitchTriggered) {
  // Define the GPIO pin for the turret limit switch.
  int turretPin = 20;
  
  // Use an atomic flag to track callback invocation.
  std::atomic<bool> turretTriggered(false);
  
  // Create a LimitSwitch instance with a 30 ms debounce threshold.
  LimitSwitch turretLimit(turretPin, 30);
  
  // Set the callback to update the flag when the switch is pressed.
  turretLimit.SetStateChangeCallback([&turretTriggered](bool pressed) {
    if (pressed) {
      turretTriggered = true;
      std::cout << "Turret Limit Switch triggered." << std::endl;
    }
  });

  std::cout << "\n----- Test Turret Limit Switch -----\n";
  std::cout << "Please trigger the Turret Limit Switch on GPIO " << turretPin << std::endl;
  std::cout << "Then press Enter to continue..." << std::endl;
  
  // Wait for the user to press Enter.
  std::cin.get();

  // Allow time for asynchronous callbacks.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Verify that the turret limit switch was triggered.
  EXPECT_TRUE(turretTriggered.load());
}
