# Unified sensor data message

std_msgs/Header header

# Encoder data
int32 left_encoder
int32 right_encoder
float64 left_velocity
float64 right_velocity

# Ultrasonic sensor
float64 ultrasonic_distance
bool ultrasonic_valid

# IMU data (if available)
bool imu_available
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Quaternion orientation

# Battery/power info
float64 battery_voltage
float64 current_draw

# Temperature sensors
float64[] temperatures
string[] temperature_labels