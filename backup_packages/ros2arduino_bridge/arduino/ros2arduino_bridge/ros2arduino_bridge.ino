#include <Arduino.h>
#include <ArduinoJson.h>  // For JSON parsing and generation

// ================================
// ROS 2 Motor Controller for Arduino Mega
// ================================
// This code implements a motor controller that communicates with ROS 2
// over serial using JSON protocol. It handles motor control and encoder feedback.
// 
// Communication Protocol:
// - Commands from ROS: {"type":"cmd_vel","left_speed":0.0,"right_speed":0.0}
// - Data to ROS: {"type":"encoder_data","left_ticks":0,"right_ticks":0,"left_speed":0.0,"right_speed":0.0}

// ===== Pin Definitions =====
// Left Motor (A)
#define LEFT_MOTOR_IN1 8     // L298N D1
#define LEFT_MOTOR_IN2 9     // L298N D2
#define LEFT_MOTOR_ENABLE 5  // L298N D9 (PWM)
#define LEFT_ENCODER_A1 2    // Encoder A channel (INT0)
#define LEFT_ENCODER_A2 3    // Encoder B channel (INT1)

// Right Motor (B)
#define RIGHT_MOTOR_IN1 10   // L298N D3
#define RIGHT_MOTOR_IN2 11   // L298N D4
#define RIGHT_MOTOR_ENABLE 6 // L298N D10 (PWM)
#define RIGHT_ENCODER_A1 18  // Encoder A channel (INT5)
#define RIGHT_ENCODER_A2 19  // Encoder B channel

// ===== Constants =====
const int ENCODER_TICKS_PER_REV = 20;  // Update this based on your encoder
const float WHEEL_RADIUS = 0.1;        // meters
const float WHEEL_CIRCUMFERENCE = 0.2 * PI; // meters
const float WHEEL_SEPARATION = 0.5;    // meters between wheels
const int MAX_MOTOR_RPM = 110;         // Maximum motor speed in RPM
const int MAX_PWM = 255;               // Maximum PWM value
const float MAX_RAD_PER_SEC = (MAX_MOTOR_RPM * 2.0 * PI) / 60.0;  // Convert RPM to rad/s
const int MOTOR_UPDATE_INTERVAL = 10;  // ms (100Hz update rate)
const unsigned long COMMAND_TIMEOUT = 500; // Stop motors if no command received after 500ms
const int JSON_BUFFER_SIZE = 256;      // Size of JSON buffer for serial communication

// PID constants - adjust these based on your motor/encoder setup
const float KP_LEFT = 2.0;   // Proportional gain
const float KI_LEFT = 0.5;   // Integral gain
const float KD_LEFT = 0.05;  // Derivative gain
const float KP_RIGHT = 2.0;  // Proportional gain
const float KI_RIGHT = 0.5;  // Integral gain
const float KD_RIGHT = 0.05; // Derivative gain

// ===== Global Variables =====
// Encoder variables
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
long lastLeftEncoderTicks = 0;
long lastRightEncoderTicks = 0;
unsigned long lastEncoderPrint = 0;

// Command processing
String inputString = "";         // A string to hold incoming data
boolean stringComplete = false;   // Whether the string is complete

// Timing
unsigned long lastMotorUpdate = 0;
unsigned long lastCommandTime = 0;
unsigned long lastPidUpdate = 0;  // For PID timing

// Motor control
float leftTargetSpeed = 0.0;    // rad/s
float rightTargetSpeed = 0.0;   // rad/s
float leftCurrentSpeed = 0.0;   // rad/s
float rightCurrentSpeed = 0.0;  // rad/s
float leftLastError = 0.0;
float rightLastError = 0.0;
float leftIntegral = 0.0;
float rightIntegral = 0.0;
bool motorsStopped = false;     // Track if motors are in a stopped state

// ===== Function Declarations =====
void updateLeftEncoder();
void updateRightEncoder();
void updateMotors();
void updatePid();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void processCommand(String command);
void printEncoderValues();
void sendEncoderData();
void serialEvent();

// ===== Setup =====
void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (only needed for boards with native USB)
  }
  
  // Send startup message
  Serial.println("DROS 2 Motor Controller Ready");
  Serial.println("DWaiting for commands...");
  
  // Set motor control pins as outputs
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Set up encoder pins with pullup resistors
  pinMode(LEFT_ENCODER_A1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A2, INPUT_PULLUP);
  
  // Attach encoder interrupts on rising/falling edges
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A1), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A1), updateRightEncoder, CHANGE);
  
  // Initialize motor control (ensure motors are stopped)
  stopMotors();
  
  // Initialize timers
  unsigned long currentTime = millis();
  lastMotorUpdate = currentTime;
  lastPidUpdate = currentTime;
  lastCommandTime = currentTime;
  
  // Print welcome and help message
  Serial.println("DROS 2 Motor Controller Ready");
  Serial.println("DCommands:");
  Serial.println("D  M<left>,<right> - Set motor speeds in rad/s");
  Serial.println("D  S - Stop motors");
  Serial.println("D  Z - Zero encoders");
  Serial.println("D  ? - This help message");
  
  // Initialize encoder positions
  lastLeftEncoderTicks = leftEncoderTicks;
  lastRightEncoderTicks = rightEncoderTicks;
}

// ===== Main Loop =====
void loop() {
  // Process any incoming serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Update motor control (PID and safety checks)
  updateMotors();
  
  // Periodically send encoder data (20Hz update rate)
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= 50) { // 50ms = 20Hz
    sendEncoderData();
    lastSendTime = currentTime;
    
    // Optional: Print debug info at a lower rate (uncomment if needed)
    // static unsigned long lastDebugTime = 0;
    // if (currentTime - lastDebugTime >= 1000) { // 1 second interval
    //   Serial.print("DDebug - Left: ");
    //   Serial.print(leftTargetSpeed, 2);
    //   Serial.print(" rad/s, Right: ");
    //   Serial.print(rightTargetSpeed, 2);
    //   Serial.println(" rad/s");
    //   lastDebugTime = currentTime;
    // }
  }
  
  // Emergency stop if no commands received
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    leftTargetSpeed = 0;
    rightTargetSpeed = 0;
  }
}

// ===== Encoder Interrupt Handlers =====
void updateLeftEncoder() {
  int a = digitalRead(LEFT_ENCODER_A1);
  int b = digitalRead(LEFT_ENCODER_A2);
  if (a == b) {
    leftEncoderTicks++;
  } else {
    leftEncoderTicks--;
  }
}

void updateRightEncoder() {
  int a = digitalRead(RIGHT_ENCODER_A1);
  int b = digitalRead(RIGHT_ENCODER_A2);
  if (a == b) {
    rightEncoderTicks++;
  } else {
    rightEncoderTicks--;
  }
}

// ===== PID Control Functions =====
void updatePid() {
  unsigned long now = millis();
  float dt = (now - lastPidUpdate) / 1000.0f;  // Convert to seconds
  
  if (dt < 0.01f) return;  // Too soon to update (minimum 10ms between updates)
  
  lastPidUpdate = now;
  
  // Calculate wheel speeds in rad/s
  long leftTicks = leftEncoderTicks - lastLeftEncoderTicks;
  long rightTicks = rightEncoderTicks - lastRightEncoderTicks;
  
  // Update encoder tracking
  lastLeftEncoderTicks = leftEncoderTicks;
  lastRightEncoderTicks = rightEncoderTicks;
  
  // Convert ticks to rad/s
  const float RAD_PER_TICK = (2.0f * PI) / ENCODER_TICKS_PER_REV;
  
  // Calculate current speeds (rad/s)
  leftCurrentSpeed = (leftTicks * RAD_PER_TICK) / dt;
  rightCurrentSpeed = (rightTicks * RAD_PER_TICK) / dt;
  
  // Update left motor PID
  float leftError = leftTargetSpeed - leftCurrentSpeed;
  leftIntegral += leftError * dt;
  
  // Anti-windup: Limit integral term
  leftIntegral = constrain(leftIntegral, -100.0f, 100.0f);
  
  float leftDerivative = (leftError - leftLastError) / dt;
  float leftOutput = KP_LEFT * leftError + 
                    KI_LEFT * leftIntegral + 
                    KD_LEFT * leftDerivative;
  leftLastError = leftError;
  
  // Update right motor PID
  float rightError = rightTargetSpeed - rightCurrentSpeed;
  rightIntegral += rightError * dt;
  
  // Anti-windup: Limit integral term
  rightIntegral = constrain(rightIntegral, -100.0f, 100.0f);
  
  float rightDerivative = (rightError - rightLastError) / dt;
  float rightOutput = KP_RIGHT * rightError + 
                     KI_RIGHT * rightIntegral + 
                     KD_RIGHT * rightDerivative;
  rightLastError = rightError;
  
  // Convert PID output to PWM (constrain to -255 to 255)
  int leftPwm = constrain((int)leftOutput, -MAX_PWM, MAX_PWM);
  int rightPwm = constrain((int)rightOutput, -MAX_PWM, MAX_PWM);
  
  // Set motor speeds
  setMotorSpeeds(leftPwm, rightPwm);
  
  // Debug output (uncomment for tuning)
  // Serial.print("DLeft: ");
  // Serial.print(leftTargetSpeed, 2);
  // Serial.print(" ");
  // Serial.print(leftCurrentSpeed, 2);
  // Serial.print(" ");
  // Serial.print(leftPwm);
  // Serial.print("\tRight: ");
  // Serial.print(rightTargetSpeed, 2);
  // Serial.print(" ");
  // Serial.print(rightCurrentSpeed, 2);
  // Serial.print(" ");
  // Serial.println(rightPwm);
}

// ===== Motor Control Functions =====
void updateMotors() {
  unsigned long now = millis();
  
  // Update PID at fixed intervals
  if (now - lastPidUpdate >= (unsigned long)MOTOR_UPDATE_INTERVAL) {
    updatePid();
  }
  
  // Emergency stop if no commands received
  if (now - lastCommandTime > COMMAND_TIMEOUT) {
    leftTargetSpeed = 0.0f;
    rightTargetSpeed = 0.0f;
    leftIntegral = 0.0f;
    rightIntegral = 0.0f;
    leftLastError = 0.0f;
    rightLastError = 0.0f;
    
    // Only send stop command once to avoid flooding the serial
    if (!motorsStopped) {
      setMotorSpeeds(0, 0);
      motorsStopped = true;
    }
  } else {
    motorsStopped = false;
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, abs(leftSpeed));
  } else if (leftSpeed < 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_ENABLE, abs(leftSpeed));
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, 0);
  }
  
  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_ENABLE, abs(rightSpeed));
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, 0);
  }
}

void stopMotors() {
  leftTargetSpeed = 0;
  rightTargetSpeed = 0;
  leftIntegral = 0;
  rightIntegral = 0;
  leftLastError = 0;
  rightLastError = 0;
  setMotorSpeeds(0, 0);
}

// ===== Command Processing =====
void processCommand(String command) {
  if (command.length() == 0) return;
  
  // Update last command time for safety timeout
  lastCommandTime = millis();
  
  // Parse JSON command
  DynamicJsonDocument doc(JSON_BUFFER_SIZE);
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.print("DJSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Check command type
  const char* type = doc["type"];
  
  if (strcmp(type, "cmd_vel") == 0) {
    // Handle motor command
    float newLeftSpeed = doc["left_speed"];  
    float newRightSpeed = doc["right_speed"];
    
    // Limit target speeds to max RPM with smooth ramping
    leftTargetSpeed = constrain(newLeftSpeed, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);
    rightTargetSpeed = constrain(newRightSpeed, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);
    
    // Debug echo
    Serial.print("DSet speeds - Left: ");
    Serial.print(leftTargetSpeed, 2);
    Serial.print(" rad/s, Right: ");
    Serial.print(rightTargetSpeed, 2);
    Serial.println(" rad/s");
  }
  else if (strcmp(type, "stop") == 0) {
    // Stop all motors
    leftTargetSpeed = 0.0f;
    rightTargetSpeed = 0.0f;
    leftIntegral = 0.0f;
    rightIntegral = 0.0f;
    leftLastError = 0.0f;
    rightLastError = 0.0f;
    Serial.println("DStopping all motors");
  }
  else if (strcmp(type, "zero_encoders") == 0) {
    // Zero encoders
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    lastLeftEncoderTicks = 0;
    lastRightEncoderTicks = 0;
    leftCurrentSpeed = 0.0f;
    rightCurrentSpeed = 0.0f;
    Serial.println("DEncoders zeroed");
  }
  else if (strcmp(type, "get_status") == 0) {
    // Send current status
    sendEncoderData();
  }
  else {
    Serial.print("DUnknown command type: ");
    Serial.println(type);
  }
}

// ===== Utility Functions =====
void sendEncoderData() {
  // Calculate wheel positions in meters
  const float METERS_PER_TICK = (2.0f * PI * WHEEL_RADIUS) / ENCODER_TICKS_PER_REV;
  float leftPosition = leftEncoderTicks * METERS_PER_TICK;
  float rightPosition = rightEncoderTicks * METERS_PER_TICK;
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  doc["type"] = "encoder_data";
  doc["left_ticks"] = leftEncoderTicks;
  doc["right_ticks"] = rightEncoderTicks;
  doc["left_speed"] = leftCurrentSpeed;
  doc["right_speed"] = rightCurrentSpeed;
  doc["left_position"] = leftPosition;
  doc["right_position"] = rightPosition;
  
  // Serialize JSON to string and send
  String output;
  serializeJson(doc, output);
  Serial.println(output);
}

void printEncoderValues() {
  Serial.print("DEncoders - Left: ");
  Serial.print(leftEncoderTicks);
  Serial.print(" (");
  Serial.print(leftCurrentSpeed, 2);
  Serial.print(" rad/s), Right: ");
  Serial.print(rightEncoderTicks);
  Serial.print(" (");
  Serial.print(rightCurrentSpeed, 2);
  Serial.println(" rad/s)");
}

// ===== Serial Event Handler =====
void serialEvent() {
  static bool inJson = false;
  static int braceCount = 0;
  static String jsonString = "";
  
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Look for start of JSON object
    if (inChar == '{') {
      inJson = true;
      braceCount = 1;
      jsonString = "{";
    }
    // Process JSON content
    else if (inJson) {
      jsonString += inChar;
      
      // Count braces to find the end of the JSON object
      if (inChar == '{') {
        braceCount++;
      } else if (inChar == '}') {
        braceCount--;
        if (braceCount == 0) {
          // Complete JSON object received
          inJson = false;
          inputString = jsonString;
          stringComplete = true;
          return;
        }
      }
      
      // Prevent buffer overflow
      if (jsonString.length() >= JSON_BUFFER_SIZE) {
        inJson = false;
        Serial.println("DError: JSON too long");
      }
    }
  }
}
