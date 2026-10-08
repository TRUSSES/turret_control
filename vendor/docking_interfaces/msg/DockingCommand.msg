# Command message for docking mechanism control

uint8 IDLE        = 0  # Deactivate: disable relay, stop servo signals, mark UNDOCKED
uint8 ACTIVATE    = 1  # Enable relay, initialize both servos to zero position
uint8 OPEN_LATCH  = 2  # Rotate latch servo (servo1/GPIO27) 90 degrees open, mark UNDOCKED
uint8 CLOSE_LATCH = 3  # Return latch servo (servo1/GPIO27) back to zero position

uint8 command
