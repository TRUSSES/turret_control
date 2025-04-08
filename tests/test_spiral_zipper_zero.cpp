#include <gtest/gtest.h>
#include "spiral_zipper.h"
#include <iostream>
#include <thread>
#include <chrono>

// This test verifies that calling SpiralZipper::Zero() retracts the motor until
// the spiral zipper limit switch is triggered and then resets the encoder count,
// resulting in an extension close to zero.
TEST(SpiralZipperTest, ZeroingWorks) {
  // Create a SpiralZipper instance.
  // Adjust the pin numbers and parameters as appropriate for your hardware.
  // Parameters: (servo_pwm_pin, servo_dir_pin, servo_enc_a, servo_enc_b, servo_enable_pin,
  //              zipper_enc_cs_pin, zipper_enc_clk_pin, zipper_enc_do_pin, limit_switch_pin,
  //              extension_per_step, debounce_threshold_ms)
  SpiralZipper zipper(22, 27, 24, 25, 4, 13, 26, 19, 16, 0.000004453125, 30);

  std::cout << "\n----- Spiral Zipper Zeroing Test -----\n";
  std::cout << "Ensure the spiral zipper is extended and the limit switch is not triggered.\n";
  std::cout << "When the test begins, the motor will start retracting slowly.\n";
  std::cout << "Please manually trigger the spiral zipper limit switch to stop the motor, then press Enter...\n";

  // Wait for the user to initiate the test.
  std::cin.get();

  // Call the Zero() method; this should retract the zipper until the limit switch is activated.
  zipper.Zero();

  // Wait briefly for any asynchronous activities to settle.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // After zeroing, the encoder count should have been reset.
  // Get the measured extension (in meters).
  double extension = zipper.GetExtension();
  std::cout << "Measured extension after zeroing: " << extension << " meters\n";

  // Verify that the extension is near zero.
  EXPECT_NEAR(extension, 0.0, 1e-6);
}
