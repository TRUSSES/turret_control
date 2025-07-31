# Turret state message
uint8 IDLE = 0
uint8 READY = 1
uint8 RUNNING = 2

uint8 state
float64 extension_length  # Current extension in meters
float64 velocity         # Current velocity in m/s
bool is_zeroed          # Whether the turret has been zeroed
string status_message   # Optional status message