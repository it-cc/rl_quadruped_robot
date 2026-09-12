# Robot state feedback message (MCU -> PC)
# Contains robot velocity command, IMU data, timestamp and error state

uint8 sequence

# Robot velocity command from remote control
float32[3] velocity  # [linear_x, linear_y, angular_z]

# IMU orientation (Euler angles in radians)
float32[3] orientation  

# IMU angular velocity (rad/s)
float32[3] angular_velocity

# IMU linear acceleration (m/s^2)
float32[3] linear_acceleration

# MCU timestamp in milliseconds
uint32 timestamp_ms

# Error state bitmask
uint32 error_state
