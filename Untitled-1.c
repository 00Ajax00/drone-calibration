#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MQUnifiedsensor.h> // Include MQUnifiedsensor library for gas sensors
#include <ESP32Servo.h> // Include ESP32Servo library for servo control
#include <ArduinoJson.h>

// Replace these with your network credentials
const char *ssid = "AJAY VIJAYAN";
const char *password = "ajay1234567890";

WebSocketsServer webSocket = WebSocketsServer(81);
MPU6050 mpu;

// Motor pins
const int MOTOR_PIN_1 = 26;
const int MOTOR_PIN_2 = 27;
const int MOTOR_PIN_3 = 14;
const int MOTOR_PIN_4 = 12;

// PID constants (adjust as needed)
const float Kp = 2.0; // Proportional gain
const float Ki = 0.1; // Integral gain
const float Kd = 1.0; // Derivative gain

// Target angles for stabilization
float targetRoll = 0.0;  // Target roll angle
float targetPitch = 0.0; // Target pitch angle

float rollIntegral = 0.0;    // Integral of roll error for PID
float pitchIntegral = 0.0;   // Integral of pitch error for PID
float lastErrorRoll = 0.0;   // Previous error for roll PID calculation
float lastErrorPitch = 0.0;  // Previous error for pitch PID calculation

// Create servo objects
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Define MQ7 and MQ135 pins
#define MQ7_PIN 34 // Connect MQ7 sensor to GPIO pin 34
#define MQ135_PIN 35 // Connect MQ135 sensor to GPIO pin 35

// Initialize MQUnifiedsensor for MQ7 and MQ135 sensors
MQUnifiedsensor MQ7("MQ7", 5.0, 10, MQ7_PIN, "MQ7"); // Adjust parameters as needed
MQUnifiedsensor MQ135("MQ135", 5.0, 10, MQ135_PIN, "MQ135"); // Adjust parameters as needed

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        // Create a JSON document to parse the payload
        StaticJsonDocument<512> doc;  // Adjust the buffer size as needed

        // Deserialize the JSON payload into the document
        DeserializationError error = deserializeJson(doc, payload, length);

        // Check for parsing errors
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
        }

        // Check if the payload contains a valid command and direction
        if (doc.containsKey("command") && doc.containsKey("direction")) {
            String command = doc["command"];
            String direction = doc["direction"];

            // Perform actions based on the command and direction received
            if (command == "move") {
                if (direction == "forward") {
                    // Code to move forward
                } else if (direction == "backward") {
                    // Code to move backward
                } else if (direction == "left") {
                    // Code to turn left
                } else if (direction == "right") {
                    // Code to turn right
                }
            }
        }
    }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Wire.begin();
  mpu.initialize();

  // Attach servo motors to pins
  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);
  motor3.attach(MOTOR_PIN_3);
  motor4.attach(MOTOR_PIN_4);

  Serial.println("Setup complete.");
}

void loop() {
  webSocket.loop();

  // Read sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate angles from gyro data
  float roll = atan2(-ay, -az) * 180.0 / M_PI;
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  // Read gas sensor values using MQUnifiedsensor functions
  float mq7Value = MQ7.readSensor(); // Use readSensor() for MQ7 sensor
  float mq135Value = MQ135.readSensor(); // Use readSensor() for MQ135 sensor

  // Send sensor data to frontend via WebSocket
  String sensorData = "{\"roll\": " + String(roll) + ", \"pitch\": " + String(pitch) + ", \"mq7\": " + String(mq7Value) + ", \"mq135\": " + String(mq135Value) + "}";
  webSocket.broadcastTXT(sensorData);
  
  // Calculate errors for PID
  float errorRoll = targetRoll - roll;
  float errorPitch = targetPitch - pitch;

  // Update integrals
  rollIntegral += errorRoll;
  pitchIntegral += errorPitch;

  // Apply PID control
  float outputRoll = Kp * errorRoll + Ki * rollIntegral + Kd * (errorRoll - lastErrorRoll);
  float outputPitch = Kp * errorPitch + Ki * pitchIntegral + Kd * (errorPitch - lastErrorPitch);

  // Update motor speeds based on PID outputs (example code)
  int motorSpeed1 = 1500 + outputPitch - outputRoll;
  int motorSpeed2 = 1500 + outputPitch + outputRoll;
  int motorSpeed3 = 1500 - outputPitch - outputRoll;
  int motorSpeed4 = 1500 - outputPitch + outputRoll;

  // Set motor speeds
  motor1.writeMicroseconds(motorSpeed1);
  motor2.writeMicroseconds(motorSpeed2);
  motor3.writeMicroseconds(motorSpeed3);
  motor4.writeMicroseconds(motorSpeed4);

  // Update last errors
  lastErrorRoll = errorRoll;
  lastErrorPitch = errorPitch;

  delay(20);  // Adjust as needed for control frequency
}