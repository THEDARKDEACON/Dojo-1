# Motor command message

std_msgs/Header header

# Motor control modes
uint8 VELOCITY_MODE = 0
uint8 PWM_MODE = 1
uint8 POSITION_MODE = 2

uint8 control_mode

# Velocity commands (m/s)
float64 left_velocity
float64 right_velocity

# PWM commands (-255 to 255)
int16 left_pwm
int16 right_pwm

# Position commands (radians)
float64 left_position
float64 right_position

# Safety limits
float64 max_velocity
float64 max_acceleration
bool emergency_stop