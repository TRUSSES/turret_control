#include "turret_controller.h"
#include <iostream>
#include <chrono>
#include <thread>
#include "config.h" 

// Constructor: It expects the config to have keys for "turret" and optionally for
// the goal parameters (defaulting to 0.5 m extension, 30° pitch, 0° yaw if not provided).
TurretController::TurretController(const YAML::Node &config)
    : turret_(config["turret"]), 
      running_(true),
      desired_extension_(config["desired_extension"] ? config["desired_extension"].as<float>() : 0.5f),
      desired_pitch_(config["desired_pitch"] ? config["desired_pitch"].as<float>() : 30.0f),
      desired_yaw_(config["desired_yaw"] ? config["desired_yaw"].as<float>() : 0.0f) {
  turret_.Init();
}

TurretController::~TurretController() {
  Stop();
}

void TurretController::Spin() {
  std::cout << "Starting main control loop..." << std::endl;
  while (running_) {
    turret_.Update();
    // Actuate turret cable with the given goal extension, pitch and yaw.
    bool goal_reached = turret_.ActuateTurretCable(desired_extension_, desired_pitch_, desired_yaw_);
    if (goal_reached) {
      std::cout << "Goal reached. Holding position." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void TurretController::Stop() {
  running_ = false;
}

void TurretController::SetGoalParameters(float desired_extension, float desired_pitch, float desired_yaw) {
  desired_extension_ = desired_extension;
  desired_pitch_ = desired_pitch;
  desired_yaw_ = desired_yaw;
}

// --- Optional main() to run the controller as a standalone application ---
// When you port to ROS2, this main() would be replaced by node initialization.


 // Assumes your singleton config class is defined here.
int main(int argc, char** argv) {
  // Load configuration (ensure your config.yaml is in your working directory).
  if (!Config::Instance().Load("config/config.yaml")) {
    std::cerr << "Failed to load configuration file. Exiting." << std::endl;
    return 1;
  }
  YAML::Node config = Config::Instance().GetConfig();

  // Create the TurretController using configuration.
  TurretController controller(config);

  // Optionally, allow the user to update goal parameters.
  float desired_extension = 0.5f;
  float desired_pitch = 30.0f;
  float desired_yaw = 0.0f;
  std::cout << "Enter desired extension (meters): ";
  std::cin >> desired_extension;
  std::cout << "Enter desired pitch angle (degrees): ";
  std::cin >> desired_pitch;
  std::cout << "Enter desired yaw angle (degrees): ";
  std::cin >> desired_yaw;
  controller.SetGoalParameters(desired_extension, desired_pitch, desired_yaw);

  // Run the main loop.
  controller.Spin();
  return 0;
}

