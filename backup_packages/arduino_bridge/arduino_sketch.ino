// Simple Arduino sketch for ROS2 communication
const int LED_PIN = 13;
const int SENSOR_PIN = A0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Read sensor
  int sensorValue = analogRead(SENSOR_PIN);
  
  // Send sensor data
  Serial.print("S");
  Serial.println(sensorValue);
  
  // Check for incoming commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'L') {
      // Read the value (0 or 1)
      while (Serial.available() == 0) { } // Wait for data
      int value = Serial.parseInt();
      digitalWrite(LED_PIN, value ? HIGH : LOW);
    }
    // Clear any remaining data in the buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  delay(100); // Small delay to prevent flooding
}
