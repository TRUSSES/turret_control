# State message published by the docking node

uint8 IDLE   = 0  # Relay off, servos not signaled
uint8 ACTIVE = 1  # Relay on, servos powered and ready

uint8 UNDOCKED = 0
uint8 DOCKING  = 1  # Automatic docking or closing the latch onto a detected dock
uint8 DOCKED   = 2  # Latch closing sequence complete; cleared by OPEN_LATCH or IDLE

uint8  state                   # Power state: IDLE / ACTIVE
uint8  docking_state           # UNDOCKED / DOCKING / DOCKED
bool   relay_active
bool   latch_open              # true when latch servo is at open (90 deg) position
bool   dock_detected           # true while the docking button is pressed (GPIO LOW)
int32  servo1_pulse_width      # Latch servo pulse width in microseconds (0 = no signal)
int32  servo2_pulse_width      # Servo 2 pulse width in microseconds (0 = no signal)
string status_message
