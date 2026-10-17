# Turret Controller

## AprilTag docking sequence

The DOCK command still uses the existing zero → READY prerequisite. It enables
the control motors and starts the controller in `include/autonomous_docking.h`:

1. **SEARCH:** hold the current extension and yaw, and pitch down until a fresh
   AprilTag pose becomes available. The default search covers the allowed pitch
   range with a 15 degrees/second pitch target rate. It waits at the bounded
   endpoint if the tag is still unavailable.
2. **PITCH_CENTER:** keep the zipper stopped and adjust pitch until the tag is
   within 2 degrees of vertical center. Yaw continues correcting lateral error.
   Capture the measured pitch only after the camera confirms centering.
   Within 25 cm, if pitch is within 1 degree of its limit and centering would
   require moving past that limit, hold the measured pitch and permit a short
   approach with fresh tracking and at most 6 degrees of vertical error. This
   lets the last few millimetres of approach reach the existing insertion handoff.
3. **APPROACH:** extend in motor-velocity bursts while holding that pitch with
   encoder feedback. Between 45 and 25 cm, smoothly reduce zipper speed from
   1.5 to 1.2 rad/s and burst duration from 2 seconds to 0.5 seconds.
   Stop the zipper after each burst,
   wait for a camera capture taken after the stop, and adjust pitch before the
   next burst. Yaw continues correcting lateral error. If vertical error exceeds
   4 degrees, stop and re-center immediately, except for the bounded near-range
   pitch-limit case above. A fresh pose at the 20 cm handoff
   ends camera alignment and starts dock preparation immediately.
4. **WAIT_FOR_TAG:** if tracking is lost before insertion, stop all motors and
   wait. Fresh tracking returns to pitch centering from measured joint positions.
   A camera delay does not trigger retraction, a sweep, or a failed attempt.
5. **DOCK_ACTIVATE / DOCK_OPEN:** the first valid fresh pose at 20 cm (5 mm depth
   deadband) captures the measured pitch and yaw. Stop extension and hold those
   angles while sending ACTIVATE to `/dockN/command`. Wait for fresh ACTIVE,
   relay-on, latch-closed feedback after the activation settling period; then
   send OPEN_LATCH and wait for fresh powered/open feedback after its settling
   period. Commands are sent once, in order, at least 2 seconds apart.
   Camera tracking is no longer
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
   each operation's settling wait. Automatic port commands are at least
   2 seconds apart. No manual latch-open command is required.
   Allow four additional power cycles if preparation fails. The contact button
   may remain pressed while the turret starts withdrawing.
2. Retract at **1.2 rad/s** while encoder feedback holds the original docking
   pitch and yaw. Take up cable based on measured zipper movement, alongside
   pitch correction, so the cable motor need not wait for pitch to drift.
   Use measured zipper travel to finish the first **10 cm**;
   elapsed time does not trigger the speed change.
3. After the 10 cm clearance, adjust pitch toward **60 degrees** while
   retracting at **2.0 rad/s** to **30 cm extension**. Keep cable takeup and
   pitch feedback active throughout retraction. If measured pitch exceeds
   **63 degrees**, stop the zipper and
   correct pitch before resuming. This check also uses the fresh encoder
   reading immediately before applying the retraction command.
4. At **30 cm extension**, adjust pitch toward **20 degrees** while continuing
   retraction at **2.0 rad/s**. Stop the zipper at **10 cm extension**
   (5 mm tolerance), finish adjusting pitch
   within 2 degrees, then change the turret state to **READY**. Continue holding
   pitch and yaw in READY with the zipper stopped. IDLE releases that hold.

If the starting extension leaves less than 10 cm above the 10 cm destination,
the slow section ends at the destination and pitch adjusts with the zipper
stopped. If clearance ends below 30 cm, go directly to the final pitch/retraction
stage. Port feedback/readiness loss during withdrawal stops extension and
holds the current target angles. Fresh powered/open feedback resumes from
the preserved clearance progress and pitch target; otherwise another bounded
power cycle can reopen the latch. Once READY is reached, holding the posture no longer depends
on port feedback. Opening feedback also reports a commanded position rather
than sensed physical latch movement.

Tuning is under `undocking` in `config/config.yaml`, with ROS names prefixed
`undocking_`. `slow_clearance_m`, `slow_velocity_rad_s`,
`fast_velocity_rad_s`, `cruise_pitch_deg`, `intermediate_extension_m`,
`target_extension_m`, `target_pitch_deg`, and `pitch_max_deg` control the motion. The pitch cap
defaults to 63 degrees and may be lowered, but cannot be raised above 63 or
the configured docking pitch limit. Both requested pitch angles must fit within
that limit. Port settling waits and `dock_retry_attempts`
are independent of the docking controller's budget. The old timed pitch-only/combined-retraction
settings have been replaced by encoder-based stages. Undocking shares the
fine pitch motor speed profile, yaw holding gains, and state freshness
parameters from `docking`. Its own `pitch_motor_velocity_limit_rad_s` defaults
to **1.2 rad/s**, allowing faster coarse corrections while retaining the fine
speed taper near the requested pitch. This limit caps the combined cable
takeup and pitch correction command.

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

Pitch and yaw share Pi3Hat's CAN receive queue. Every command, including MIT
entry/exit and motor zeroing, now routes all received replies to the motor named
by the payload ID on that transport and bus. A yaw reply received during a pitch
transaction updates yaw feedback instead of being discarded. Commands and
feedback access are serialized across both motors. Mode-entry logs report that
the command was sent; they do not claim that the motor acknowledged it.
Shutdown uses ROS's deferred signal handling, with hardware stopping in ordinary
cleanup after the zeroing worker has been aborted and joined. A signal handler
never tries to re-enter the locked CAN transport.

All tuning is in `config/config.yaml` under `docking`. These keys replace the
older stage/acquire/final-insertion-distance settings. ROS overrides use the
same names prefixed with `docking_`, such as `docking_approach_burst_duration_sec`.
Docking's `pitch_max_deg` defaults to **70 degrees**. Undocking retains its
separate **63-degree** ceiling and its existing requirement that starting pitch
is within that ceiling; a dock completed above 63 degrees currently prevents
UNDOCK from starting.
`target_dock_id` selects `/dockN/state` and `/dockN/command` independently of
`turret_id`; restart the node after changing that selection. `target_tag_frame`
selects this turret's
own `/vision/end_effector_tag_pose_camera/<tag_frame>` and
`/vision/tag_visible/<tag_frame>` topics; restart after changing it too.
The default target is `tag36h11_25` (family `36h11`, ID 25). The camera detector
and pose bridge use this single docking ID so testing-area VIO tags cannot
become docking targets. The active detector's physical size remains 0.056 m
(5.6 cm); the compatibility detector YAML in this package uses the same size.
Copy the matching camera package configuration and restart its nodes as well.
`camera_frame` must match the AprilTag pose frame. The bridge's aggregate
preferred/fallback stream remains available for visualization and cannot
switch the autonomous controller to another turret's tag.
The target dock and tag parameters are read-only after startup; other docking
tuning changes take effect on the next DOCK request.

Far-range yaw alignment stops once the tag's camera-x error is **3 cm or less**, using
`docking.lateral_deadband_m: 0.030`. An active correction is canceled at that
threshold and the measured yaw becomes a fixed hold, while pitch and extension
continue through the normal sequence. After reaching that window, new yaw
corrections require error **greater than 5 cm**, configured by
`yaw_resume_lateral_error_m: 0.050`, plus the existing fresh-camera observation
checks. Error between 3 and 5 cm therefore keeps the settled hold instead of
repeatedly starting and stopping near the cutoff. A new DOCK request starts
coarse alignment again if its initial error is outside 3 cm. Once a fresh visual
pose enters **45 cm**, switch to the near-range **1 cm stop / 2 cm restart**
window (`near_yaw_lateral_deadband_m` / `near_yaw_lateral_resume_m`). Entering
that region clears any earlier far-range acceptance, so a residual 2–3 cm error
gets another correction. The tighter mode stays active for the rest of that
DOCK request, avoiding repeated mode changes from noisy depth estimates.
The legacy `lateral_tolerance_m` does not gate insertion, but must remain at
least as large as `lateral_deadband_m`.

When a coarse correction is needed, yaw requests one full 3-degree position step from the
measured yaw at the start of each correction. The current signed camera
lateral error supplies the direction directly; the core's older yaw setpoint
cannot veto a correction after measured yaw passes it. There is no cumulative
yaw travel limit relative to the heading at docking start. Drive toward that
fixed target using the same velocity command as
teleop: `yaw_motor_velocity_rad_s` defaults to **3.5 rad/s**, with the
direction selected from the camera error. This matches the measured working
speed above the mechanism's friction threshold. Position holding uses
`yaw_position_kp` = 160 and `yaw_position_kd` = 2 between bursts, during
insertion, and during automatic undocking.
A target stays fixed until it is reached within
`yaw_target_deadband_deg` (0.2 degrees) or `yaw_adjustment_timeout_sec`
(2 seconds) expires. The configured MIT position range gives approximately
0.167 degrees per feedback increment; the 3-degree target spans about eighteen
increments. New step sizes taper to **1.5 degrees** by 25 cm; an active target
stays fixed while its step finishes. The controller then holds the measured angle, waits at
least `yaw_observation_pause_sec` (0.2 seconds), and requires a camera capture
taken after that stop before starting another correction. The settled position
target stays fixed during observation and while the camera is centered;
ordinary encoder changes do not move the hold target. Cached poses cannot
queue repeated yaw steps. Pitch search, dock preparation and final insertion
hold their captured yaw without camera alignment corrections.
The pitch controller's post-burst camera wait disables new yaw steps until
a capture taken after the burst stop arrives, while allowing an active fixed
position target to finish. It cannot replace that target with the pitch
controller's older yaw setpoint. Actual tracking loss, contact, insertion
handoff, or a CAN interruption still cancels the active correction.
A yaw-step timeout logs the commanded step, measured movement, motor torque
feedback, and drive error flag at most every five seconds, making an
unresponsive motor visible without repeated per-step INFO messages.

The earlier position-only correction requested zero motor velocity. Correction
now uses MIT velocity control (`kp=0`, `kd=0.3`) and zero feed-forward torque,
matching `SetYawVelocity` in teleop. `yaw_velocity_pulse_sec` defaults to
**0.015 seconds**, tapering to **0.008 seconds** for new corrections by 25 cm.
The adapter uses the selected pulse duration for the entire active target.
Actual yaw movement speed remains **3.5 rad/s** so short adjustments still
overcome friction. Each short burst polls position/CAN feedback until the fixed
target is reached/crossed or the pulse duration expires, then brakes at zero
velocity. The controller validates the braking reply before holding its
measured angle or starting another pulse.
Repeated bursts can finish the same target; they cannot accumulate farther
targets. The callback runs the bounded burst synchronously, normally up to
15 ms plus its final CAN braking commands. A zero-speed high-damping command
brakes before the current measured position becomes the held angle. No high-speed command is intentionally
left running between callbacks. Small valid backward feedback changes are
treated as position error toward the same fixed target, so recoil does not
cancel the step or start a camera-recovery cycle. The original 2-second step
deadline still applies; repeated recoil cannot extend it. A position change
over 5 degrees inside a pulse stops the pulse and requires confirmation, even
if the later stopped reply returns near the prior angle. Drive errors remain
errors. The target is bounded when the controller creates it; backward movement
does not cause a separate remaining-distance fault. If a new reply has already
crossed the target before the next pulse starts, stop that pulse and complete
the existing step on the next update.
Velocity is limited to 5 rad/s, step size to 5 degrees, and burst duration to
50 ms. The configured 3.5 rad/s is separate from `max_yaw_rate_deg_s`, which
ramps the camera-guided request and does not set physical motor velocity.
The obsolete `yaw_correction_kp`/`yaw_correction_kd` tuning keys are replaced
by the velocity and pulse settings. The MIT encoder rounds
and bounds each field to `2^bits - 1`, matching its decoder; maximum gains no
longer overflow into adjacent command fields.
Motor feedback must be recent (`yaw_feedback_timeout_sec`, default 0.25 seconds)
before starting a correction. DOCK and UNDOCK acquire their initial yaw reference
through stopped, zero-velocity queries for up to 0.5 seconds, requiring a newly
decoded motor reply rather than merely a recent cached value. This startup
acquisition interval is separate from the feedback-age limit. Dock-state messages
queued during acquisition are received before checking the initial dock snapshot;
the code does not renew freshness without an actual message. During a burst,
feedback is checked after each CAN command with an age window of at most 5 ms.
Missing replies brake the current pulse. Up to two stopped CAN queries require
a newly decoded reply with a sequence newer than the corresponding query and
an age within 5 ms. If that succeeds, validate the stopped angle and continue
the same target without canceling the step or waiting for another camera
capture. If stopped feedback is still missing, pause motion rather than
permanently fail the attempt. Each subsequent control tick stops and sends neutral yaw queries;
a new valid reply resumes the controller automatically and cancels an
interrupted nudge. Phase deadlines and port settling/retry clocks pause with
motion, while camera timestamps and dock-state freshness remain in real ROS
time. The 2-second port-command spacing is preserved across these pauses.
Visual approach also needs an image captured after the communication stop;
insertion and port preparation retain their camera-independent joint holds.
Fresh yaw is read before capturing the insertion reference.
The yaw controller observes the latest valid CAN reply after every burst and
braking hold, so its next comparison includes the movement it just commanded.
A change over 5 degrees no longer fails the attempt immediately: stop the
zipper, keep pitch held, brake yaw without commanding an unconfirmed angle,
and confirm the position using two distinct new CAN replies within 1 degree.
Re-reading a cached reply cannot satisfy this check. Visual alignment also
waits for a camera capture taken after that confirmation and the observation
pause before resuming; search and insertion use CAN confirmation alone.
After CAN confirmation, the motor holds that fixed confirmed position while
waiting for the camera. On recovery, confirmed measured yaw also replaces any
old frozen hold target. Missing CAN replies stop motion while awaiting recovery;
actual drive faults and invalid encoder values remain errors. Recovery is
reported as a docking pause and does not
set the permanent motor-command failure flag. The motor protocol's feedback
angle is used directly; there is no additional yaw gearing conversion.

Throttled messages distinguish a recovered CAN reply delay from small backward
motion that keeps the same correction target. A large discontinuity retains
the separate position-confirmation warning; small recoil no longer produces
repeated "Yaw feedback recovered" camera waits.

If startup still receives no yaw feedback, the failure includes motor ID, CAN
bus, accepted-reply count and age, bus/own-ID frame counts, rejected frame sizes,
and the last received payload/CAN IDs. This distinguishes no motor-11 reply
from replies that arrive but cannot be decoded. The decoder still requires the
existing eight-byte reply format; rejected or cached data never renews freshness.

Separate yaw zeroing is not required for docking. Place the turret roughly
facing the dock; DOCK reads its current yaw for the initial hold, and subsequent
camera corrections use the measured heading without a relative travel cutoff.
The former `yaw_limit_from_start_deg` setting has been removed. Normal turret
zeroing homes extension and pitch.
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
`centering_start_tolerance_deg` accepts **2 degrees** of vertical camera error
before an extension burst; `centering_stop_tolerance_deg` stops an active burst
above **4 degrees**. Once accepted, hold the measured pitch instead of waiting
for exact equality with a requested joint angle. The separate motor controller
already accepts 0.5 degrees of joint-pitch error; that deadband is unchanged.
`approach_burst_duration_sec` sets the far-range maximum uninterrupted extension
duration (default 2 seconds). `approach_slowdown_start_m` (45 cm) and
`approach_slowdown_end_m` (25 cm) define a shared smooth distance profile.
`near_approach_burst_duration_sec` (0.5 seconds) and
`near_zipper_motor_velocity_rad_s` (1.2 rad/s) define the near-range approach.
The working 1.2 rad/s zipper minimum is retained. `zipper_motor_velocity_rad_s` and
`insertion_velocity_rad_s` are direct motor angular speeds. Approach no longer
uses changing position targets or the old `extension_gain`,
`max_extension_rate_m_s`, and `max_extension_step_m` settings.
Pitch targets now adapt to camera alignment error. Below
`pitch_fine_error_deg` (3 degrees), centering uses `fine_pitch_rate_deg_s`
(5 degrees/second) and `fine_pitch_step_deg` (2 degrees of lead over measured
pitch). Between that error and `pitch_coarse_error_deg` (8 degrees), both limits
blend smoothly toward `max_pitch_rate_deg_s` (15 degrees/second) and
`max_pitch_step_deg` (6 degrees). Targets cannot lead farther than the remaining
signed angular correction visible in the current image. Search uses
`search_pitch_rate_deg_s` (15 degrees/second). These are target rates and lead
limits; actual joint speed depends on cable geometry and load. The distance
profile also scales pitch target rates and lead down to `near_pitch_scale`
(0.5), giving smaller centering adjustments near the dock. The physical cable
motor's fine-speed profile and working minimum remain in effect.

The shared cable-motor controller in `include/docking_pitch_control.h` also
adapts to measured joint-pitch error. It stops inside the existing 0.5-degree
deadband. Outside it, speed rises smoothly from the working 0.20 rad/s minimum
to `pitch_motor_fine_velocity_rad_s` (0.25 rad/s) at
`pitch_motor_fine_error_deg` (2 degrees), then toward
`pitch_motor_velocity_limit_rad_s` (0.80 rad/s) at
`pitch_motor_coarse_error_deg` (6 degrees). Larger errors use the coarse cap;
small errors slow down as the motor approaches the requested angle. This
profile applies to docking search/centering, pitch holds during extension and
insertion, undocking retraction, and the final READY posture. Undocking uses
its separate 1.2 rad/s coarse cap. Normal manual
control and zeroing retain their existing controllers. During extension and
retraction, encoder feedback pays out or takes up cable to restore the held
pitch when the mechanism drifts.
Pitch and pre-insertion extension bounds remain active. Search, approach,
and tracking pauses have no deadline. Latch-wait has its own timeout after
contact has stopped the zipper and captured the joint hold.
`dock_activate_settle_sec` (2 seconds), `dock_open_settle_sec` (1 second),
and `dock_deactivate_settle_sec` (2 seconds) cover the docking port's operations.
After each wait,
a later fresh state message must confirm the expected flags before advancing;
an old ACTIVE or open flag alone is insufficient. If the port's servo timing is
changed, increase these waits to cover it. `dock_prepare_timeout_sec` defaults
to 5 seconds per preparation stage or state-recovery wait and must exceed all
settling periods. It bounds stopped waits, not insertion.
`dock_command_interval_sec` defaults to 2 seconds in both `docking` and
`undocking`. Within each automatic run, all port commands respect this minimum
spacing, including retries and a contact-triggered close cycle. If contact
arrives before the delay expires, stop the zipper immediately and replace any
queued activation/opening with the latch-closing sequence. Keep pitch/yaw held
while waiting. These delays are nonblocking: joint control and state callbacks
continue running. A queued command does not consume its operation timeout or
settling wait; those start when the command is emitted. Manual port commands
are independent of this turret-side spacing.
`dock_retry_attempts` defaults to four additional IDLE/ACTIVATE/OPEN_LATCH cycles
across the entire DOCK attempt; zero disables power-cycle retries. The count
does not reset when preparation succeeds or a heartbeat returns. A retry needs
a fresh unoccupied state, so an offline dock node or a lost link cannot be
repaired by blindly switching its relay. A brief insertion heartbeat gap can
resume on confirmed powered/open feedback without consuming a retry.
After contact, a separate closing budget uses the same retry count with
IDLE/ACTIVATE/CLOSE_LATCH. `latch_close_settle_sec` defaults to 1 second,
and `latch_closed_pulse_width_us` defaults to 1000; match the latter to the
docking node's `servo1_zero_us`. Retained DOCKED alone cannot bypass the
forced close cycle. Log messages describe command confirmation rather than
claiming a physical latch measurement.

Approach uses a direct zipper motor command, tapering from 1.5 to 1.2 rad/s near
the dock. Final insertion uses
1.4 rad/s while maintaining the handoff pitch and stopping on button contact.
Tracking loss, contact and the
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
camera lateral error, measured/target yaw, yaw motor velocity, torque feedback,
and the drive error flag without additional high-rate prints. Separate yaw
messages report correction and post-motion camera-wait transitions at DEBUG
level; correction timeouts are warnings throttled to five seconds.
Routine centering/burst transitions are also DEBUG messages;
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
the docking, undocking, adaptive pitch, cable takeup, and CAN reply-routing tests through CTest.
The standalone pitch profile test source is `tests/test_docking_pitch_control.cpp`.
Measured travel and cable command tests are in `tests/test_undocking_cable_control.cpp`.
Reply ownership and registration tests are in `tests/test_cubemars_reply_router.cpp`.
Near-range approach, pitch-limit progress, and yaw refinement sources are
`tests/test_near_docking_approach.cpp` and `tests/test_near_docking_yaw.cpp`.

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
