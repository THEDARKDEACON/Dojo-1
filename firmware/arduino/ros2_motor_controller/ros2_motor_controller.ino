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
//   E<left_ticks>,<right_ticks>,<left_rad>,<right_rad>,<left_pos>,<right_pos>\n
// ================================

// ===== Pin Definitions (Arduino Mega) =====
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
const float WHEEL_RADIUS           = 0.033f;       // meters; must match arduino_driver.wheel_radius
const float WHEEL_CIRCUMFERENCE    = 2.0f * PI * WHEEL_RADIUS; // meters; must match arduino_driver.wheel_circumference
const float WHEEL_SEPARATION       = 0.20f;        // meters; must match arduino_driver.wheel_base

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
  unsigned long now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt < (MOTOR_UPDATE_MS / 1000.0f)) return;
  lastPidMs = now;

  // Compute current wheel speeds from encoder deltas
  long dL = leftEncoderTicks - lastLeftTicks;
  long dR = rightEncoderTicks - lastRightTicks;
  lastLeftTicks  = leftEncoderTicks;
  lastRightTicks = rightEncoderTicks;

  // ticks -> rad: 2π / ticksPerRev
  const float RAD_PER_TICK = (2.0f * PI) / ENCODER_TICKS_PER_REV;
  leftRad  = (dL * RAD_PER_TICK) / dt;
  rightRad = (dR * RAD_PER_TICK) / dt;

  // PID for left
  float eL = leftTargetRad - leftRad;
  leftInt  += eL * dt;  leftInt = constrain(leftInt, -100.0f, 100.0f);
  float dEL = (eL - leftPrev) / dt;  leftPrev = eL;
  float uL = KP_LEFT * eL + KI_LEFT * leftInt + KD_LEFT * dEL;

  // PID for right
  float eR = rightTargetRad - rightRad;
  rightInt += eR * dt; rightInt = constrain(rightInt, -100.0f, 100.0f);
  float dER = (eR - rightPrev) / dt; rightPrev = eR;
  float uR = KP_RIGHT * eR + KI_RIGHT * rightInt + KD_RIGHT * dER;

  // Convert controller output to PWM
  int leftPWM  = clampPWM((int)uL);
  int rightPWM = clampPWM((int)uR);

  setMotorPWM(leftPWM, rightPWM);
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
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (ch >= 32 && ch <= 126) {
      cmdBuffer += ch;
      if (cmdBuffer.length() > 64) { // avoid runaway
        cmdBuffer = "";
        Serial.println(F("DError: Command too long"));
      }
    }
  }
}

// ===== Setup & Loop =====
void setup() {
  // IO
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  pinMode(LEFT_ENCODER_A1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A1), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A1), isrRight, CHANGE);

  stopMotors();

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("DDojo ROS2 Motor Controller Ready"));

  unsigned long now = millis();
  lastPidMs = now;
  lastTxMs = now;
  lastCmdMs = now;
}

void loop() {
  readSerial();

  // Update control at fixed rate
  if (millis() - lastPidMs >= MOTOR_UPDATE_MS) {
    updatePID();
  }

  // Telemetry @20Hz
  if (millis() - lastTxMs >= TX_PERIOD_MS) {
    sendTelemetry();
    lastTxMs = millis();
  }

  // Safety timeout
  if (millis() - lastCmdMs > CMD_TIMEOUT) {
    if (!motorsStopped) {
      stopMotors();
    }
  }
}
