# Turret state message
uint8 IDLE = 0
uint8 READY = 1
uint8 RUNNING = 2
uint8 TELEOP = 3
uint8 TELEOP_ZERO = 4
uint8 DOCKING = 5
uint8 UNDOCKING = 6

uint8 state
float64 extension_length  # Current spiral zipper extension in meters
float64 velocity         # Current spiral zipper velocity in m/s
float64 pitch_angle      # Current pitch angle in radians (from encoder after zeroing)
float64 yaw_angle        # Current yaw angle in radians (from motor feedback)
bool is_zeroed          # Whether the turret has been fully zeroed (SZ + pitch + yaw)
bool sz_zeroed          # Whether spiral zipper is zeroed
bool pitch_zeroed       # Whether pitch is zeroed
bool yaw_zeroed         # Whether yaw is zeroed
float64 force           # Current force reading in Newtons
bool load_cell_ready    # Whether load cells are calibrated and ready
string status_message   # Optional status message
