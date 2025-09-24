// ================================
// Dojo ROS 2 Motor Controller (Arduino Mega)
// Protocol compatible with robot_hardware/arduino_driver.py when
// arduino_driver.use_speed_commands = true
// 
// Commands from ROS 2:
//   M<left_rad>,<right_rad>\n   Set wheel target speeds (rad/s)
//   S\n                          Stop motors
//   Z\n                          Zero encoders
//   ?\n                          Help
// 
// Telemetry to ROS 2 (20 Hz):
//   E<left_ticks>,<right_ticks>,<left_rad>,<right_rad>,<left_pos>,<right_pos>\n// 
// Error Responses:
//   E:Buffer overflow\n
// ================================
// Safety Features:
// - Watchdog timer (500ms)
// - Command validation
// - Buffer overflow protection
// - Motor limits
// ================================

#include <avr/wdt.h> // Watchdog timer
#include <Servo.h>
#include <std_msgs/Bool.h>   // for /servo_cmd topic

// ===== Pin Definitions (Arduino Mega) =====
// ===== Servo config =====
const int SERVO_PIN = 12;   // set to your servo signal pin
Servo dropServo;
bool servoActive = false;
unsigned long servoOpenTime = 0;

// Left Motor (A)
#define LEFT_MOTOR_IN1 8      // L298N IN1
#define LEFT_MOTOR_IN2 9      // L298N IN2
#define LEFT_MOTOR_ENABLE 5   // L298N ENA (PWM)
#define LEFT_ENCODER_A1 2     // Encoder A (INT0)
#define LEFT_ENCODER_A2 3     // Encoder B (INT1)

// Right Motor (B)
#define RIGHT_MOTOR_IN1 10    // L298N IN3
#define RIGHT_MOTOR_IN2 11    // L298N IN4
#define RIGHT_MOTOR_ENABLE 6  // L298N ENB (PWM)
#define RIGHT_ENCODER_A1 18   // Encoder A (INT5)
#define RIGHT_ENCODER_A2 19   // Encoder B (INT4)

// ===== Robot Geometry & Encoders =====
const int   ENCODER_TICKS_PER_REV = 20;            // Must match ROS param arduino_driver.encoder_ticks_per_rev
const float WHEEL_RADIUS           = 0.030f;       // meters; must match arduino_driver.wheel_radius
const float WHEEL_CIRCUMFERENCE    = 2.0f * PI * WHEEL_RADIUS; // meters; must match arduino_driver.wheel_circumference
const float WHEEL_SEPARATION       = 0.26f;        // meters; must match arduino_driver.wheel_base

// ===== Motor/Controller Limits =====
const int   MAX_PWM             = 255;  // 8-bit PWM
const int   MOTOR_UPDATE_MS     = 10;   // 100 Hz
const unsigned long TX_PERIOD_MS= 50;   // 20 Hz telemetry
const unsigned long CMD_TIMEOUT = 500;  // Stop if no command after 500 ms

// ===== PID Gains (tune experimentally) =====
const float KP_LEFT  = 2.0f;
const float KI_LEFT  = 0.5f;
const float KD_LEFT  = 0.05f;
const float KP_RIGHT = 2.0f;
const float KI_RIGHT = 0.5f;
const float KD_RIGHT = 0.05f;

// ===== State =====
volatile long leftEncoderTicks  = 0;
volatile long rightEncoderTicks = 0;
long lastLeftTicks  = 0;
long lastRightTicks = 0;
bool systemError = false;

float leftTargetRad  = 0.0f;
float rightTargetRad = 0.0f;
float leftRad        = 0.0f;  // measured
float rightRad       = 0.0f;  // measured
float leftInt  = 0.0f,  rightInt  = 0.0f;
float leftPrev = 0.0f,  rightPrev = 0.0f;

unsigned long lastPidMs = 0;
unsigned long lastTxMs  = 0;
unsigned long lastCmdMs = 0;
bool motorsStopped = true;

// ===== Serial Command Buffer =====
String cmdBuffer;

// Callback for servo command
void servoCmdCallback(const std_msgs::Bool& msg) {
  if (msg.data && !servoActive) {
    dropServo.write(90);                // open (adjust angle if needed)
    Serial.println("Servo opened via /servo_cmd");
    servoActive = true;
    servoOpenTime = millis();           // record when it opened
  }
}

// ===== Interrupts =====
void isrLeft() {
  int a = digitalRead(LEFT_ENCODER_A1);
  int b = digitalRead(LEFT_ENCODER_A2);
  leftEncoderTicks += (a == b) ? 1 : -1;
}

void isrRight() {
  int a = digitalRead(RIGHT_ENCODER_A1);
  int b = digitalRead(RIGHT_ENCODER_A2);
  rightEncoderTicks += (a == b) ? 1 : -1;
}

// ===== Motor Driver =====
static inline int clampPWM(int x) {
  if (x > MAX_PWM) return MAX_PWM;
  if (x < -MAX_PWM) return -MAX_PWM;
  return x;
}

void setMotorPWM(int leftPWM, int rightPWM) {
  // Left
  if (leftPWM > 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, leftPWM);
  } else if (leftPWM < 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_ENABLE, -leftPWM);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, 0);
  }

  // Right
  if (rightPWM > 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, rightPWM);
  } else if (rightPWM < 0) {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_ENABLE, -rightPWM);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, 0);
  }
}

void stopMotors() {
  leftTargetRad = 0.0f;
  rightTargetRad = 0.0f;
  leftInt = rightInt = 0.0f;
  leftPrev = rightPrev = 0.0f;
  setMotorPWM(0, 0);
  motorsStopped = true;
  Serial.println(F("DStopping all motors"));
}

// ===== Control (PID on wheel angular velocity) =====
void updatePID() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  // Skip if in error state
  if (systemError) {
    stopMotors();
    return;
  }
  
  // Limit update rate
  if (now - lastUpdate < MOTOR_UPDATE_MS) return;
  lastUpdate = now;
  
  // Check for command timeout
  if (now - lastCmdMs > CMD_TIMEOUT) {
    stopMotors();
    motorsStopped = true;
    return;
  }
  
  // Skip if no target speed
  if (leftTargetRad == 0 && rightTargetRad == 0) {
    leftInt = 0;
    rightInt = 0;
    return;
  }
  
  // Calculate time delta (in seconds)
  float dt = (now - lastPidMs) / 1000.0f;
  lastPidMs = now;
  
  // Calculate errors
  float leftError = leftTargetRad - leftRad;
  float rightError = rightTargetRad - rightRad;
  
  // Update integrals with anti-windup
  leftInt += leftError * dt;
  rightInt += rightError * dt;
  
  // Limit integral terms to prevent windup
  leftInt = constrain(leftInt, -MAX_PWM/KI_LEFT, MAX_PWM/KI_LEFT);
  rightInt = constrain(rightInt, -MAX_PWM/KI_RIGHT, MAX_PWM/KI_RIGHT);
  
  // Calculate derivatives
  float leftDeriv = (leftRad - leftPrev) / dt;
  float rightDeriv = (rightRad - rightPrev) / dt;
  
  // Store current values for next iteration
  leftPrev = leftRad;
  rightPrev = rightRad;
  
  // Calculate PID outputs
  float leftOutput = KP_LEFT * leftError + KI_LEFT * leftInt - KD_LEFT * leftDeriv;
  float rightOutput = KP_RIGHT * rightError + KI_RIGHT * rightInt - KD_RIGHT * rightDeriv;
  
  // Safety checks
  if (isnan(leftOutput) || isnan(rightOutput) || 
      isinf(leftOutput) || isinf(rightOutput)) {
    systemError = true;
    Serial.println("E:PID output error");
    return;
  }
  
  // Apply motor outputs with safety limits
  setMotorPWM(
    constrain((int)leftOutput, -MAX_PWM, MAX_PWM),
    constrain((int)rightOutput, -MAX_PWM, MAX_PWM)
  );
}

// ===== Telemetry =====
void sendTelemetry() {
  // Wheel positions (meters)
  float leftPos  = leftEncoderTicks  * (WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV);
  float rightPos = rightEncoderTicks * (WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV);

  Serial.print('E');
  Serial.print(leftEncoderTicks);
  Serial.print(',');
  Serial.print(rightEncoderTicks);
  Serial.print(',');
  Serial.print(leftRad, 4);
  Serial.print(',');
  Serial.print(rightRad, 4);
  Serial.print(',');
  Serial.print(leftPos, 6);
  Serial.print(',');
  Serial.println(rightPos, 6);
}

// ===== Command Parsing =====
void handleCommand(const String &command) {
  if (command.length() == 0) return;
  lastCmdMs = millis();

  char c = command.charAt(0);
  String params = command.substring(1);

  switch (c) {
    case 'M': {
      int comma = params.indexOf(',');
      if (comma > 0) {
        float l = params.substring(0, comma).toFloat();
        float r = params.substring(comma + 1).toFloat();
        leftTargetRad  = l;
        rightTargetRad = r;
        motorsStopped = false;
        Serial.print(F("DSet speeds rad/s - L:")); Serial.print(leftTargetRad, 3);
        Serial.print(F(" R:")); Serial.println(rightTargetRad, 3);
      }
    } break;

    case 'S':
      stopMotors();
      break;

    case 'Z':
      noInterrupts();
      leftEncoderTicks = rightEncoderTicks = 0;
      interrupts();
      lastLeftTicks = lastRightTicks = 0;
      leftRad = rightRad = 0.0f;
      Serial.println(F("DEncoders zeroed"));
      break;

    case '?':
      Serial.println(F("D=== Dojo ROS2 Motor Controller ==="));
      Serial.println(F("DCommands:"));
      Serial.println(F("D  M<left>,<right>   Set wheel speeds (rad/s)"));
      Serial.println(F("D  S                 Stop motors"));
      Serial.println(F("D  Z                 Zero encoders"));
      Serial.println(F("D  ?                 Help"));
      Serial.print(F("DGeometry: R=")); Serial.print(WHEEL_RADIUS, 3);
      Serial.print(F(" m, Base=")); Serial.println(WHEEL_SEPARATION, 3);
      Serial.print(F("DEncoder ticks/rev: ")); Serial.println(ENCODER_TICKS_PER_REV);
      break;

    default:
      Serial.print(F("DUnknown command: ")); Serial.println(command);
      break;
  }
}

void readSerial() {
  static String inputBuffer = "";
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        if (validateCommand(inputBuffer)) {
          handleCommand(inputBuffer);
        } else {
          Serial.println("E:Invalid command");
        }
        inputBuffer = "";
      }
    } 
    // Add character to buffer if it won't cause overflow
    else if (inputBuffer.length() < 64) {  // Max command length
      inputBuffer += c;
    }
    // Handle buffer overflow
    else {
      // Clear buffer and send error
      inputBuffer = "";
      Serial.println("E:Buffer overflow");
      // Skip until end of line
      while (Serial.available() > 0 && Serial.peek() != '\n' && Serial.peek() != '\r') {
        Serial.read();
      }
    }
  }
}

// Helper function to check if string is a valid number
bool isNumber(const String &s) {
  if (s.length() == 0) return false;
  bool hasDecimal = false;
  
  for (unsigned int i = 0; i < s.length(); i++) {
    if (s[i] == '-' && i == 0) continue;  // Allow negative sign at start
    if (s[i] == '.') {
      if (hasDecimal) return false;  // Only one decimal allowed
      hasDecimal = true;
      continue;
    }
    if (!isDigit(s[i])) return false;
  }
  return true;
}

// Validate command format
bool validateCommand(const String &cmd) {
  if (cmd.length() < 1) return false;
  char cmdType = cmd[0];
  
  switch (cmdType) {
    case 'M':  // Motor command: M<float>,<float>
      {
        int commaPos = cmd.indexOf(',');
        if (commaPos == -1) return false;
        
        String leftStr = cmd.substring(1, commaPos);
        String rightStr = cmd.substring(commaPos + 1);
        
        return isNumber(leftStr) && isNumber(rightStr);
      }
      
    case 'S':  // Stop
    case 'Z':  // Zero encoders
    case '?':  // Help
      return cmd.length() == 1;
      
    default:
      return false;
  }
}

// ===== Setup & Loop =====
void setup() {

  // --- Servo setup ---
  dropServo.attach(SERVO_PIN);
  dropServo.write(0);   // start closed

  // --- ROS2 subscriber ---
  nh.subscribe<std_msgs::Bool>("servo_cmd", servoCmdCallback);
  
  // Disable watchdog during setup
  wdt_disable();
  
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  
  // Initialize encoder pins
  pinMode(LEFT_ENCODER_A1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A2, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A1), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A1), isrRight, CHANGE);
  
  // Initialize serial
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for serial port to connect
  
  // Initial stop
  stopMotors();
  
  // Initialize watchdog (500ms timeout)
  wdt_enable(WDTO_500MS);
  
  // Print help
  Serial.println("Dojo ROS 2 Motor Controller Ready");
  Serial.println("Commands:");
  Serial.println("M<left_rad>,<right_rad> - Set speeds (rad/s)");
  Serial.println("S - Stop motors");
  Serial.println("Z - Zero encoders");
  Serial.println("? - This help");
  
  systemError = false;
}

void loop() {
  // --- Servo auto-close after 5s ---
  if (servoActive && (millis() - servoOpenTime > 5000UL)) {
    dropServo.write(0);   // close
    Serial.println("Servo auto-closed after 5s");
    servoActive = false;
  }
  // ---------------------------------

  nh.spinOnce();   // process ROS messages

  
  // Reset watchdog timer
  wdt_reset();
  
  // Skip processing if in error state
  if (systemError) {
    stopMotors();
    delay(100);
    return;
  }
  
  // Read serial commands
  readSerial();
  
  // Update PID control
  updatePID();
  
  // Send telemetry periodically
  unsigned long currentTime = millis();
  if (currentTime - lastTxMs > TX_PERIOD_MS) {
    sendTelemetry();
    lastTxMs = currentTime;
  }
  
  // Small delay to prevent busy-waiting
  delay(1);
}
