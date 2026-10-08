# Turret Controller

## AprilTag docking sequence

The DOCK command still uses the existing zero → READY prerequisite. It enables
the control motors and starts the controller in `include/autonomous_docking.h`:

1. **SEARCH:** hold the current extension and yaw, and pitch down until a fresh
   AprilTag pose becomes available. The default search covers the allowed pitch
   range at 8 degrees/second.
2. **ALIGN:** correct camera lateral error with yaw and vertical error with
   pitch while holding extension.
3. **APPROACH:** extend toward the camera while continuing yaw and pitch
   corrections. Pause extension when alignment leaves its tolerance.
4. **RECOVERY:** if tracking is lost before the 10 cm handoff, stop all motors,
   retract 3 cm, then sweep pitch ±10 degrees around the lost-tracking pitch.
   A fresh pose returns the controller to alignment after retraction finishes.
5. **INSERT:** at 10 cm with alignment satisfied (5 mm depth deadband), hold yaw
   and insert at fixed zipper motor speed until docking contact. **There is no
   insertion travel limit or insertion timer.** Camera loss during this phase
   does not interrupt insertion. Stale docking-port feedback stops the motors.
6. **WAIT_FOR_LATCH / COMPLETE:** contact or the port's retained `DOCKING` state
   stops all motors immediately; `DOCKED` confirms the port's closing sequence
   completed. The port already closes its latch automatically on contact.

`/turretN/state.status_message` reports the current phase and failure reason,
and the command-station UI displays it. IDLE stops the turret; a new DOCK request
starts a new attempt. A fresh unoccupied dock state is required when starting.
The existing separate dock OPEN_LATCH control is still used before UNDOCK.

All tuning is in `config/config.yaml` under `docking`. These keys replace the
older stage/acquire/final-insertion-distance settings. ROS overrides use the
same names prefixed with `docking_`, such as `docking_recovery_retract_m`.
`target_dock_id` selects `/dockN/state` independently of `turret_id`; restart the
node after changing that subscription. `target_tag_frame` selects this turret's
own `/vision/end_effector_tag_pose_camera/<tag_frame>` and
`/vision/tag_visible/<tag_frame>` topics; restart after changing it too.
`camera_frame` must match the AprilTag pose frame. The bridge's aggregate
preferred/fallback stream remains available for visualization and cannot
switch the autonomous controller to another turret's tag.
The target dock and tag parameters are read-only after startup; other docking
tuning changes take effect on the next DOCK request.

Calibrate `yaw_correction_sign` against camera lateral motion on the actual
robot. Positive pitch is assumed to move down from the homed position, and
`pitch_down_sign` controls the search direction. The camera goal defaults to
x = 0, y = 3 cm, z = 10 cm. `zipper_motor_velocity_rad_s` and
`insertion_velocity_rad_s` are motor angular speeds, while
`max_extension_rate_m_s` is a Cartesian planning rate. Pitch/yaw and pre-insertion
extension bounds remain active; search/recovery and latch-wait timeouts are
separate from the unlimited contact-driven insertion.

The docking message definitions are bundled in `vendor/docking_interfaces`.
Copy the complete `turret_control` folder to the Pi workspace's `src` directory,
then run `colcon build --packages-select turret_control --cmake-clean-cache`.
The build generates the existing `docking/msg/DockingState` and
`docking/msg/DockingCommand` types, so the turret can receive the docking port's
state using only this folder. Keep the bundled message definitions in sync with
the docking port whenever its message layout changes.

The bridge rejects detections older than 0.5 seconds, and the turret separately
checks the detection stamp and receipt age; the camera and turret ROS clocks
must be synchronized.

The controller can be tested without ROS or hardware:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iinclude \
  tests/test_autonomous_docking.cpp -o /tmp/test_autonomous_docking
/tmp/test_autonomous_docking
```

With ROS installed, `colcon test --packages-select turret_control` also runs
the same controller tests through CTest.

![Build: Manual](https://img.shields.io/badge/Build-Manual-informational?style=for-the-badge)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg?style=for-the-badge)](https://en.cppreference.com/w/cpp/17)


![Project Banner](https://drive.google.com/file/d/1vOpIJWYqArkjfUXqExw0h_km0_sw6xsK/view)  



## Table of Contents

- [About The Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)


## About The Project

The Turret Controller System is a modular C++ project designed for controlling a turret system on a Raspberry Pi. The system incorporates the following elements:
- **Hardware Control Loop**: Manages turret orientation along with a spiral zipper actuator for precise cable control.
- **YAML Configuration**: All hardware pin mappings and parameters are stored in an external configuration file, enabling easy updates without code changes.
- **Testing Framework**: Unit and integration tests are implemented using GoogleTest to verify hardware interactions and component functionality.
- **Future Integration**: The project is structured for a smooth transition to a ROS2 node environment.

### Built With

- [C++17](https://en.cppreference.com/w/cpp/17)
- [CMake](https://cmake.org/)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) – For GPIO handling on Raspberry Pi  
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) – For parsing YAML configuration files  
- [GoogleTest](https://github.com/google/googletest) – For unit and integration tests

## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

- **Raspberry Pi** running Ubuntu 22.04 LTS server
- A C++17 compiler (GCC, Clang, etc.)
- [CMake 3.10+](https://cmake.org/)
- [pigpio Library](http://abyz.me.uk/rpi/pigpio/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (or using CMake’s FetchContent)
- Git

## Project Structure
```
turret_control/
├── include/
│   ├── config.h
│   ├── encoder.h
│   ├── limit_switch.h
│   ├── servo_city_motor.h
│   ├── spiral_zipper.h
│   ├── turret.h
│   └── turret_controller.h    
|           ## Header Files
├── src/
│   ├── config.cpp
│   ├── encoder.cpp
│   ├── limit_switch.cpp
│   ├── servo_city_motor.cpp
│   ├── spiral_zipper.cpp
│   ├── turret.cpp
│   ├── turret_controller.cpp
│   └── cubemars_control.cpp  
|           ## Source Files
├── tests/
│   ├── test_limit_switches.cpp
│   └── test_spiral_zipper_zero.cpp
|           ## Test Cases - GoogleTest
├── config.yaml                
|           # YAML configuration for pin mapping
├── CMakeLists.txt             # CMake build script
└── README.md                  # Project documentation

```

## 🛠️ Installation & Configuration

### 📦 Prerequisites

Ensure you have the following installed on your system:

- **C++17 compiler** (e.g. `g++`, `clang++`)
- **CMake ≥ 3.10**
- **pigpio** library for GPIO control
- **yaml-cpp** library for parsing YAML files
- **GoogleTest** (included via CMake)

On a Raspberry Pi (Ubuntu 22.04), install dependencies:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libpigpio-dev libyaml-cpp-dev
```
### 📁 Clone the Repository

``` bash
git clone https://github.com/your-org/turret_control.git
cd turret_control
```

## ⚙️ Configuration
All hardware pin mappings and system parameters are specified in config.yaml.
Make sure the file exists in your project root directory.

config file syntax:

``` yaml
spiral_zipper:
  motor_pwm_pin: 18
  encoder_a_pin: 5
  encoder_b_pin: 6
  limit_switch_pin: 17
  # ...
```

> 📝 Note: This file doesnt need to change unless you change some pin mappings or if you want to tune the encoder or gear ratio settings


## 🧱 Build the Project
Create a build directory:
``` bash
mkdir build && cd build
```
Configure and build:

```bash
cmake ..
make
```

This builds both the main executable (turret) and the test binary (turret_tests).
