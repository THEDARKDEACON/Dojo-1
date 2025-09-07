#include <ros2arduino.h>

// Define pins
#define LEFT_MOTOR_PWM 5     // PWM pin for left motor speed control
#define LEFT_MOTOR_DIR 6     // Direction pin for left motor
#define RIGHT_MOTOR_PWM 9    // PWM pin for right motor speed control
#define RIGHT_MOTOR_DIR 10   // Direction pin for right motor

// Motor control parameters
#define MAX_PWM 255          // Maximum PWM value (0-255)
#define MIN_PWM 30           // Minimum PWM value to overcome friction (adjust based on your motors)
#define DEADBAND 0.05        // Minimum input value to consider (to prevent motor jitter)
#define MAX_ACCEL 0.5        // Maximum acceleration (0.0 - 1.0) per control cycle
#define RAMP_RATE 0.5        // Rate of acceleration (0.0 - 1.0)

// ROS2 Node
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t encoder_pub;
sensor_msgs__msg__JointState encoder_msg;

// Subscribers
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// Timer for publishing sensor data
rcl_timer_t timer;

// Variables for sensors and motor control
float left_encoder = 0.0;
float right_encoder = 0.0;
float left_motor_output = 0.0;   // Current motor output (-1.0 to 1.0)
float right_motor_output = 0.0;  // Current motor output (-1.0 to 1.0)
unsigned long last_motor_update = 0;  // For timing motor updates

// Function prototypes
void setupROS2();
void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdVelCallback(const void *msg_in);
void updateSensors();
void controlMotors(float linear, float angular);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  
  // Initialize ROS2
  setupROS2();
  
  // Initialize IMU (if available)
  // imu.begin();
  
  // Initial motor stop
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  
  // Wait for ROS2 to be ready
  delay(1000);
}

void loop() {
  // Spin ROS2
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // Update sensors
  updateSensors();
  
  // Small delay to prevent watchdog issues
  delay(10);
}

void setupROS2() {
  // Initialize allocator
  allocator = rcl_get_default_allocator();
  
  // Initialize support structure
  rclc_support_init(&support, 0, NULL, &allocator);
  
  // Create node
  rclc_node_init_default(&node, "arduino_node", "", &support);
  
  // Create publishers
  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data");
    
  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "wheel_encoders");
    
  // Create subscriber
  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
    
  // Create timer for sensor publishing
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),  // 10 Hz
    timerCallback);
    
  // Create executor
  rclc_executor_init(&executor, &support.context, 10, &allocator);
  
  // Add timer to executor
  rclc_executor_add_timer(&executor, &timer);
  
  // Add subscriber to executor
  rclc_executor_add_subscription(
    &executor,
    &cmd_vel_sub,
    &cmd_vel_msg,
    &cmdVelCallback,
    ON_NEW_DATA);
    
  // Initialize messages
  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__JointState__init(&encoder_msg);
geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  
  // Initialize joint names for encoder message
  const char* joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
  encoder_msg.name.size = 2;
  encoder_msg.name.data = (rosidl_runtime_c__String*)malloc(2 * sizeof(rosidl_runtime_c__String));
  for (int i = 0; i < 2; i++) {
    rosidl_runtime_c__String__assign(&encoder_msg.name.data[i], joint_names[i]);
  }
  encoder_msg.position.size = 2;
  encoder_msg.position.data = (double*)malloc(2 * sizeof(double));
  encoder_msg.velocity.size = 2;
  encoder_msg.velocity.data = (double*)malloc(2 * sizeof(double));
  encoder_msg.effort.size = 0;
  encoder_msg.effort.data = NULL;
  
}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  
  if (timer != NULL) {
    // Update sensor data
    updateSensors();
    
    // Publish IMU data (if available)
    // imu_msg.header.stamp = micros_rolling_over_32();
    // rcl_publish(&imu_pub, &imu_msg, NULL);
    
    // Publish encoder data
    encoder_msg.header.stamp = micros_rolling_over_32();
    encoder_msg.position.data[0] = left_encoder;
    encoder_msg.position.data[1] = right_encoder;
    rcl_publish(&encoder_pub, &encoder_msg, NULL);
    
  }
}

void cmdVelCallback(const void *msg_in) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msg_in;
  
  // Update the last command timestamp
  static unsigned long last_cmd_time = 0;
  last_cmd_time = millis();
  
  // Extract linear and angular velocities
  // Clamp values to [-1.0, 1.0] range
  float linear = constrain(twist->linear.x, -1.0, 1.0);
  float angular = constrain(twist->angular.z, -1.0, 1.0);
  
  // Scale down the input if the combined command would exceed max PWM
  float left = linear - angular;
  float right = linear + angular;
  float max_command = max(fabs(left), fabs(right));
  
  if (max_command > 1.0) {
    linear /= max_command;
    angular /= max_command;
  }
  
  // Control motors based on the received velocities
  controlMotors(linear, angular);
  
  // Optional: Publish motor commands for debugging
  /*
  static rcl_publisher_t motor_cmd_pub;
  static std_msgs__msg__Float32MultiArray motor_cmd_msg;
  static bool publisher_initialized = false;
  
  if (!publisher_initialized) {
    motor_cmd_msg.data.capacity = 2;
    motor_cmd_msg.data.size = 2;
    motor_cmd_msg.data.data = (float*)malloc(2 * sizeof(float));
    rclc_publisher_init_default(
      &motor_cmd_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "motor_commands");
    publisher_initialized = true;
  }
  
  motor_cmd_msg.data.data[0] = left_motor_output;
  motor_cmd_msg.data.data[1] = right_motor_output;
  rcl_publish(&motor_cmd_pub, &motor_cmd_msg, NULL);
  */
}

void updateSensors() {
  // Read encoders (simulated for now)
  static unsigned long last_encoder_update = 0;
  if (millis() - last_encoder_update > 50) {  // 20 Hz update
    left_encoder += 0.1 * random(-10, 10);
    right_encoder += 0.1 * random(-10, 10);
    last_encoder_update = millis();
  }
  
  // Update IMU (simulated for now)
  // if (imu.available()) {
  //   imu_msg.orientation.x = imu.readFloatAccelX();
  //   imu_msg.orientation.y = imu.readFloatAccelY();
  //   imu_msg.orientation.z = imu.readFloatAccelZ();
  //   // Add gyro and other IMU data as needed
  // }
}

void controlMotors(float linear, float angular) {
  // Safety check - stop motors if no valid command received recently
  static unsigned long last_cmd_time = 0;
  if (millis() - last_cmd_time > 500) {  // 500ms timeout
    left_motor_output = 0.0;
    right_motor_output = 0.0;
    analogWrite(LEFT_MOTOR_PWM, 0);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    return;
  }
  
  // Calculate target speeds using differential drive kinematics
  float target_left = linear - angular;
  float target_right = linear + angular;
  
  // Apply deadband to prevent motor jitter
  if (fabs(target_left) < DEADBAND) target_left = 0.0;
  if (fabs(target_right) < DEADBAND) target_right = 0.0;
  
  // Limit the rate of change (acceleration)
  float delta_left = target_left - left_motor_output;
  float delta_right = target_right - right_motor_output;
  
  // Apply acceleration limiting
  delta_left = constrain(delta_left, -MAX_ACCEL, MAX_ACCEL);
  delta_right = constrain(delta_right, -MAX_ACCEL, MAX_ACCEL);
  
  // Update motor outputs with acceleration limiting
  left_motor_output += delta_left * RAMP_RATE;
  right_motor_output += delta_right * RAMP_RATE;
  
  // Map from [-1.0, 1.0] to [0, 255] with deadband handling
  int left_pwm = 0;
  int right_pwm = 0;
  
  if (fabs(left_motor_output) > 0) {
    left_pwm = (int)(fabs(left_motor_output) * (MAX_PWM - MIN_PWM) + MIN_PWM);
    left_pwm = constrain(left_pwm, 0, MAX_PWM);
  }
  
  if (fabs(right_motor_output) > 0) {
    right_pwm = (int)(fabs(right_motor_output) * (MAX_PWM - MIN_PWM) + MIN_PWM);
    right_pwm = constrain(right_pwm, 0, MAX_PWM);
  }
  
  // Set motor directions (LOW = forward, HIGH = reverse for most motor drivers)
  digitalWrite(LEFT_MOTOR_DIR, left_motor_output >= 0 ? LOW : HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, right_motor_output >= 0 ? LOW : HIGH);
  
  // Apply PWM to motors
  analogWrite(LEFT_MOTOR_PWM, left_pwm);
  analogWrite(RIGHT_MOTOR_PWM, right_pwm);
  
  // Debug output (uncomment for troubleshooting)
  /*
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 100) {
    Serial.print("L: ");
    Serial.print(left_pwm);
    Serial.print(" R: ");
    Serial.println(right_pwm);
    last_debug = millis();
  }
  */
}

