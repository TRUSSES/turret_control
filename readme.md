# Turret Controller

![Build: Manual](https://img.shields.io/badge/Build-Manual-informational?style=for-the-badge)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg?style=for-the-badge)](https://en.cppreference.com/w/cpp/17)


![Project Banner](https://www.modlabupenn.org/wp-content/uploads/2017/12/DSCF1529-227x300.jpg)  
*C++ Control Code for the Spiral Zipper Turret.*


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

