# Turret Controller

## AprilTag docking sequence

The DOCK command still uses the existing zero → READY prerequisite. It enables
the control motors and starts the controller in `include/autonomous_docking.h`:

1. **SEARCH:** hold the current extension and yaw, and pitch down until a fresh
   AprilTag pose becomes available. The default search covers the allowed pitch
   range with a 5 degrees/second pitch target rate. It waits at the bounded
   endpoint if the tag is still unavailable.
2. **PITCH_CENTER:** keep the zipper stopped and adjust pitch until the tag is
   within 1 degree of vertical center. Yaw continues correcting lateral error.
   Capture the measured pitch only after the camera confirms centering.
3. **APPROACH:** extend at fixed zipper motor speed for up to 2 seconds while
   holding that pitch with encoder feedback. Stop the zipper after each burst,
   wait for a camera capture taken after the stop, and adjust pitch before the
   next burst. Yaw continues correcting lateral error. If vertical error exceeds
   4 degrees, stop and re-center immediately. A fresh pose at the 20 cm handoff
   ends camera alignment and starts dock preparation immediately.
4. **WAIT_FOR_TAG:** if tracking is lost before insertion, stop all motors and
   wait. Fresh tracking returns to pitch centering from measured joint positions.
   A camera delay does not trigger retraction, a sweep, or a failed attempt.
5. **DOCK_ACTIVATE / DOCK_OPEN:** the first valid fresh pose at 20 cm (5 mm depth
   deadband) captures the measured pitch and yaw. Stop extension and hold those
   angles while sending ACTIVATE to `/dockN/command`. Wait for fresh ACTIVE,
   relay-on, latch-closed feedback after the activation settling period; then
   send OPEN_LATCH and wait for fresh powered/open feedback after its settling
   period. Commands are sent once, in order. Camera tracking is no longer
   required during preparation. Each preparation stage has a 5-second timeout;
   a timeout enters dock recovery with extension stopped.
6. **INSERT:** hold the same pitch and yaw captured before dock preparation.
   The distance handoff takes priority over further pitch/yaw correction and
   post-burst camera waits, including while centering pitch. Pitch uses
   turret encoder feedback to pay out or take up cable as extension changes,
   maintaining that captured joint angle. Insert at fixed zipper
   motor speed until docking contact. **There is no insertion travel limit or
   insertion timer.** Camera loss during this phase does not interrupt insertion.
   Stale docking-port feedback stops extension and enters dock recovery while
   keeping the captured pitch/yaw.
7. **DOCK_RECOVER / DOCK_DEACTIVATE:** wait up to 5 seconds for a new fresh port
   state. If insertion was interrupted and the port is still active, powered,
   open and unoccupied, resume insertion without cycling power. Otherwise,
   fresh UNDOCKED/button-released feedback permits an IDLE → ACTIVATE →
   OPEN_LATCH retry. Confirm IDLE/relay-off after the deactivation settling wait,
   then perform the same activation/opening checks before inserting again.
   Allow up to four additional reset cycles across the whole attempt. All
   recovery/preparation waits stop extension and hold the original joint angles;
   camera tracking is not required. If feedback never returns or the retry
   budget is exhausted, stop and report the reason.
8. **LATCH_DEACTIVATE / LATCH_ACTIVATE / WAIT_FOR_LATCH:** contact, `DOCKING`,
   or `DOCKED` immediately stops zipper extension and captures measured pitch
   and yaw. Hold those angles while forcing IDLE → ACTIVATE → CLOSE_LATCH,
   even when the port already reports DOCKED. Each operation waits for new
   feedback after settling. Closure requires ACTIVE, relay-on, latch-closed,
   and the configured closed servo pulse. A failed acknowledgement enters
   **LATCH_RECOVER** and can repeat the cycle up to four additional times,
   with a separate budget from approach/preparation retries. Contact is sticky:
   a released button cannot resume insertion or send OPEN_LATCH. Missing
   feedback stops the retry sequence after its bounded recovery wait.
9. **COMPLETE:** the close command was acknowledged; stop all turret motors.
   Port flags and pulse widths report commanded servo state. There is no latch
   position sensor, so DOCKED/`latch_open: false` cannot prove physical closure.
   The forced cycle addresses the observed false-success case, but a motor
   that fails while still acknowledging commands cannot be detected in software.
   If the button releases during that forced cycle, the port may remain
   UNDOCKED because IDLE clears its retained contact state. The turret keeps
   its own contact context and still completes on the close acknowledgement.

## Undocking sequence

UNDOCK uses the zeroed joint encoders and `/dockN/state`; it does not need the
camera. Capture the docked extension, pitch, and yaw at the start.

1. Hold the turret stationary and send IDLE → ACTIVATE → OPEN_LATCH. Confirm
   fresh relay-off, then ACTIVE/relay-on, then powered/open feedback after
   each operation's settling wait. No manual latch-open command is required.
   Allow four additional power cycles if preparation fails. The contact button
   may remain pressed while the turret starts withdrawing.
2. Retract at **1.2 rad/s** while encoder feedback holds the original docking
   pitch and yaw. Use measured zipper travel to finish the first **15 cm**;
   elapsed time does not trigger the speed change.
3. After the 15 cm clearance, capture the measured pitch, capped at
   **63 degrees**, and hold that angle while retracting at **1.8 rad/s** to
   **30 cm extension**. Use joint encoder feedback to adjust the cable as the
   zipper retracts. If measured pitch exceeds the cap, stop the zipper and
   correct pitch before resuming. This check also uses the fresh encoder
   reading immediately before applying the retraction command.
4. At **30 cm extension**, adjust pitch toward **20 degrees** while continuing
   retraction at **1.8 rad/s**. Stop the zipper at **10 cm extension**
   (5 mm tolerance), finish adjusting pitch
   within 2 degrees, then change the turret state to **READY**. Continue holding
   pitch and yaw in READY with the zipper stopped. IDLE releases that hold.

If the starting extension leaves less than 15 cm above the 10 cm destination,
the slow section ends at the destination and pitch adjusts with the zipper
stopped. If clearance ends below 30 cm, go directly to the final pitch/retraction
stage. Port feedback/readiness loss during withdrawal stops extension and
holds the current target angles. Fresh powered/open feedback resumes from
the preserved clearance progress and captured pitch; otherwise another bounded
power cycle can reopen the latch. Once READY is reached, holding the posture no longer depends
on port feedback. Opening feedback also reports a commanded position rather
than sensed physical latch movement.

Tuning is under `undocking` in `config/config.yaml`, with ROS names prefixed
`undocking_`. `slow_clearance_m`, `slow_velocity_rad_s`,
`fast_velocity_rad_s`, `intermediate_extension_m`, `target_extension_m`,
`target_pitch_deg`, and `pitch_max_deg` control the motion. The pitch cap
defaults to 63 degrees and may be lowered, but cannot be raised above 63 or
the configured docking pitch limit. Port settling waits and `dock_retry_attempts`
are independent of the docking controller's budget. The old timed pitch-only/combined-retraction
settings have been replaced by encoder-based stages. Undocking shares the
existing pitch motor velocity cap, yaw holding gains, and state freshness
parameters from `docking`.

During automatic zeroing, the homing worker performs the encoder reads and
zipper motor updates; the ROS update timer only checks whether that worker has
finished. Encoder samples and reference resets are serialized. Both switches
must release, extension must back off, and pitch must finish within 12 ±5
degrees before zeroing reports READY. The pitch encoder already tracks rollover
continuously; a reading such as 370 degrees after homing is rejected rather
than wrapped into an apparently valid angle. Docking refreshes joint readings
at startup and reports measured pitch/extension and their limits if it refuses
the starting pose.

`/turretN/state.status_message` reports the current phase and failure reason,
and the command-station UI displays it. IDLE stops the turret; a new DOCK request
starts a new attempt. A fresh unoccupied dock state is required when starting.
UNDOCK now opens the latch through its own preparation sequence.
Before a new docking test, keep the docking Pi and node running, with the button
released and the port unoccupied. The port may remain IDLE with its servo relay
off through zeroing, search and approach: manual ACTIVATE/OPEN_LATCH before the
run is no longer needed. Enable the separate servo battery's output; software
relay commands cannot wake an external battery that has shut itself off.
The docking port retains DOCKED after a button release;
ACTIVATE powers/initializes the servos but does not clear that retained state.
Use the existing IDLE control on an empty port to reset it to UNDOCKED with the
relay off, or OPEN_LATCH on an active, empty port to reset it to UNDOCKED. Startup
rejection messages identify both the contact level and retained docking state.

The bundled Pi3Hat receive loop honors forced CAN polling even when no reply
is pending. Surplus replies never make the expected-reply counters negative.
CAN cycles also leave GPIO 13 untouched: that pin is the configured pitch
encoder data input. Each encoder explicitly initializes its CS/clock outputs
and data input at startup.

All tuning is in `config/config.yaml` under `docking`. These keys replace the
older stage/acquire/final-insertion-distance settings. ROS overrides use the
same names prefixed with `docking_`, such as `docking_approach_burst_duration_sec`.
`target_dock_id` selects `/dockN/state` and `/dockN/command` independently of
`turret_id`; restart the node after changing that selection. `target_tag_frame`
selects this turret's
own `/vision/end_effector_tag_pose_camera/<tag_frame>` and
`/vision/tag_visible/<tag_frame>` topics; restart after changing it too.
`camera_frame` must match the AprilTag pose frame. The bridge's aggregate
preferred/fallback stream remains available for visualization and cannot
switch the autonomous controller to another turret's tag.
The target dock and tag parameters are read-only after startup; other docking
tuning changes take effect on the next DOCK request.

Yaw docking corrections request one full 0.5-degree position step from the
measured yaw at the start of each correction, clamped to the relative yaw
bounds. The camera-guided core request supplies the direction; its small
initial magnitude no longer reduces the step to almost the completion
deadband. `yaw_position_kp`
defaults to 160 and `yaw_position_kd` to 2; automatic undocking uses the same
holding gains. A target stays fixed until it is reached within
`yaw_target_deadband_deg` (0.15 degrees) or `yaw_adjustment_timeout_sec`
(0.5 seconds) expires. The controller then holds the measured angle, waits at
least `yaw_observation_pause_sec` (0.2 seconds), and requires a camera capture
taken after that stop before starting another correction. Cached poses cannot
queue repeated yaw steps. Pitch search, dock preparation and final insertion
hold their captured yaw without camera alignment corrections.
The pitch controller's post-burst camera wait also disables new yaw steps until
a capture taken after the burst stop arrives. A yaw-step timeout logs the
commanded step and measured movement at most every five seconds, making an
unresponsive motor visible without repeated per-step INFO messages.

These are position corrections with zero commanded yaw velocity. A 4 rad/s
velocity command would represent about 2.3 degrees in one 10 ms control tick,
so raising the velocity command does not provide the requested small steps.
Motor feedback must be recent (`yaw_feedback_timeout_sec`, default 0.25 seconds)
before using the stronger position gain. If that age window expired during a
callback delay, the controller stops all motors and sends up to three neutral
yaw queries before checking again. Fresh yaw is read before capturing the
insertion reference; an interrupted nudge is canceled before motion resumes.
Missing replies after those queries, large feedback jumps
or measured yaw outside its relative motion bound stop docking. The motor protocol's feedback angle is used
directly; there is no additional yaw gearing conversion in this controller.

Separate yaw zeroing is not required for docking. Place the turret roughly
facing the dock; DOCK reads its current yaw and applies the existing relative
30-degree correction bound. Normal turret zeroing homes extension and pitch.
The optional `/turretN/zero_yaw` service defines the current heading as zero;
it does not move the turret to a physical reference. To use it, align and stop
yaw first, then call the service in IDLE or TELEOP_ZERO.

Calibrate `yaw_correction_sign` against camera lateral motion on the actual
robot. Positive pitch is assumed to move down from the homed position, and
`pitch_down_sign` controls the search direction. The camera goal defaults to
x = 0, y = 0, z = 20 cm. `goal_camera_y_m` can shift the vertical target if the
camera mounting requires an offset. Centering uses
`atan2(goal_camera_y_m - camera_y_m, camera_z_m)`; angular tolerances remain
meaningful as the camera-to-tag distance changes. The start/stop tolerances
prevent repeated centering/extension transitions from small pose noise.
`approach_burst_duration_sec` sets the maximum uninterrupted extension duration
(default 2 seconds). `zipper_motor_velocity_rad_s` and
`insertion_velocity_rad_s` are direct motor angular speeds. Approach no longer
uses changing position targets or the old `extension_gain`,
`max_extension_rate_m_s`, and `max_extension_step_m` settings.
Pitch/yaw target rates default to 5 degrees/second. The cable motor has a separate
`pitch_motor_velocity_limit_rad_s` (default 0.35 rad/s). Centering and insertion
retain the existing 0.20 rad/s minimum correction outside their 0.5-degree joint
deadband. Approach uses the same encoder feedback while holding the captured
pitch. Actual joint speed depends on the mechanism. During extension, pitch
corrections restore the held angle if the mechanism drifts. The tighter camera
centering tolerance continues pitch adjustment beyond the old 2-degree window;
it does not impose an arbitrary downward angle or camera offset.
Pitch/yaw and pre-insertion extension bounds remain active. Search, approach,
and tracking pauses have no deadline. Latch-wait has its own timeout after
contact has stopped the zipper and captured the joint hold.
`dock_activate_settle_sec` (1.0 seconds), `dock_open_settle_sec` (0.6 seconds),
and `dock_deactivate_settle_sec` (0.3 seconds) cover the docking port's operations.
After each wait,
a later fresh state message must confirm the expected flags before advancing;
an old ACTIVE or open flag alone is insufficient. If the port's servo timing is
changed, increase these waits to cover it. `dock_prepare_timeout_sec` defaults
to 5 seconds per preparation stage or state-recovery wait and must exceed all
settling periods. It bounds stopped waits, not insertion.
`dock_retry_attempts` defaults to four additional IDLE/ACTIVATE/OPEN_LATCH cycles
across the entire DOCK attempt; zero disables power-cycle retries. The count
does not reset when preparation succeeds or a heartbeat returns. A retry needs
a fresh unoccupied state, so an offline dock node or a lost link cannot be
repaired by blindly switching its relay. A brief insertion heartbeat gap can
resume on confirmed powered/open feedback without consuming a retry.
After contact, a separate closing budget uses the same retry count with
IDLE/ACTIVATE/CLOSE_LATCH. `latch_close_settle_sec` defaults to 0.6 seconds,
and `latch_closed_pulse_width_us` defaults to 1000; match the latter to the
docking node's `servo1_zero_us`. Retained DOCKED alone cannot bypass the
forced close cycle. Log messages describe command confirmation rather than
claiming a physical latch measurement.

Approach and insertion use a direct 1.2 rad/s zipper motor command, the measured
minimum that moves the current spiral zipper. Tracking loss, contact and the
approach extension bound can interrupt a burst before its deadline; the bound
does not apply to final insertion. Update
the Pi's YAML alongside the code; older YAML or ROS parameter overrides take
precedence over these defaults. Both speeds can still be tuned for the hardware.

The docking message definitions are bundled in `vendor/docking_interfaces`.
Copy the complete `turret_control` folder to the Pi workspace's `src` directory,
then run `colcon build --packages-select turret_control --cmake-clean-cache`.
The build generates the existing `docking/msg/DockingState` and
`docking/msg/DockingCommand` types, so the turret can receive the docking port's
state and send preparation commands using only this folder. Keep the bundled
message definitions in sync with the docking port whenever its message layout
changes.

The bridge and turret allow detections up to 1.0 second old by default to
accommodate the measured camera/delivery delay. Set camera launch
`max_transform_age_sec` and turret YAML `pose_timeout_sec` to the same value;
older explicit launch overrides still take precedence. Both ends preserve and
check the original image timestamp. Repeated cached poses do not renew the
time since the last new camera measurement. The turret receives the latest
vision sample with a one-message best-effort queue to avoid processing a backlog
of old poses. The camera and turret ROS clocks must be synchronized.
The turret logs rejected docking poses with their timestamp age and reason
while camera guidance is active. `detection_age` is measured from the original
image timestamp; `receipt_age` is the time since a newer capture reached the
turret. Republishing a cached transform does not refresh either measurement.
At the distance handoff, one INFO message reports the measured tag depth and
held pitch/yaw and dock activation. After that, stale camera poses do not produce
docking rejection warnings or pause preparation/insertion. The latch-opening
and insertion-start transitions each produce one INFO message.
`lateral_tolerance_m` remains accepted for older
configurations but no longer gates this handoff; yaw still corrects lateral
error during approach.
Docking feedback logs show vertical angular error, measured/target pitch, and
whether the zipper is stopped for centering/new camera feedback or extending
a burst with pitch held, along with the commanded motor speed, at most once
every five seconds. They also show
camera lateral error, measured/target yaw and yaw motor velocity; separate yaw
messages report correction, timeout and post-motion camera-wait transitions
at DEBUG level. Routine centering/burst transitions are also DEBUG messages;
search, dock preparation/retries, insertion, contact, completion and failure
remain visible at INFO. Recovery warnings include dock-state receipt age and
the retry count; each transition logs once rather than on every control tick.
Tracking-loss logs include visibility, frame names, detection age, and time since
the last accepted pose message and are throttled to five seconds, as are pose
rejection warnings. The live state/status topics retain every update.
Compare these ages with the camera bridge's
`pose_age` log to separate camera processing delay from downstream delivery delay
or cross-Pi clock offset.

The controller can be tested without ROS or hardware:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iinclude \
  tests/test_autonomous_docking.cpp -o /tmp/test_autonomous_docking
/tmp/test_autonomous_docking
```

With ROS installed, `colcon test --packages-select turret_control` also runs
the docking and undocking controller tests through CTest. The new undocking
test source is `tests/test_autonomous_undocking.cpp`.

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
