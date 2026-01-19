# Teleop command message for direct velocity control of turret motors
# All velocities are commanded directly without position feedback

float64 spiral_zipper_velocity  # Desired velocity in m/s (positive = extend, negative = retract)
float64 pitch_velocity          # Desired velocity in rad/s
float64 yaw_velocity            # Desired velocity in rad/s
