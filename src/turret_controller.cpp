#include "turret_controller.h"
#include <chrono>
#include <iostream>
#include <thread>

TurretController::TurretController(int socket)
    : turret_(socket),
      running_(true),
      desired_extension_(0.5f),  // Default values (can be overwritten via SetGoalParameters)
      desired_pitch_(30.0f),
      desired_yaw_(0.0f) {
  turret_.Init();
}

TurretController::~TurretController() {
  Stop();
}

void TurretController::SetGoalParameters(float desired_extension, float desired_pitch, float desired_yaw) {
  desired_extension_ = desired_extension;
  desired_pitch_ = desired_pitch;
  desired_yaw_ = desired_yaw;
}

void TurretController::Spin() {
  std::cout << "Starting main control loop..." << std::endl;
  while (running_) {
    turret_.Update();
    bool goal_reached = turret_.ActuateTurretCable(desired_extension_, desired_pitch_, desired_yaw_);
    if (goal_reached) {
      std::cout << "Goal reached. Holding position." << std::endl;
      // Once reached, keep updating the turret to hold the position.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void TurretController::Stop() {
  running_ = false;
}

// Main function integrated directly into the turret_controller module.
// In the future, this could be adapted into a ROS2 node with rclcpp::spin.
int main(int argc, char* argv[]) {
  // For demonstration, use a dummy socket value (replace with your actual CAN socket descriptor).
  int socket = 0;
  TurretController controller(socket);

  float desired_extension = 0.5f;  // meters (example)
  float desired_pitch = 30.0f;     // degrees (example)
  float desired_yaw = 0.0f;        // degrees (not implemented yet)

  std::cout << "Enter desired extension (meters): ";
  std::cin >> desired_extension;
  std::cout << "Enter desired pitch angle (degrees): ";
  std::cin >> desired_pitch;
  std::cout << "Enter desired yaw angle (degrees): ";
  std::cin >> desired_yaw;

  controller.SetGoalParameters(desired_extension, desired_pitch, desired_yaw);

  // This Spin() method will continuously run the control loop, similar to a ROS2 node.
  controller.Spin();

  return 0;
}
