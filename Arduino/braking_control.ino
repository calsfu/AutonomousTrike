#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BRAKE_LIGHT_PIN 9  // Digital pin connected to MOSFET gate

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel assignments
const int brakeServo1Channel = 0; // Channel for Servo 1
const int brakeServo2Channel = 1; // Channel for Servo 2

// Define servo positions
const int servo1ReleasePosition = 65;  // Position to release the brake
const int servo1BrakePosition = 130;   // Position to apply the brake
const int servo2ReleasePosition = 90;  // Position to release the brake
const int servo2BrakePosition = 45;   // Position to apply the brake

// Define PCA9685 pulse width range
const int servoMin = 150; // Minimum pulse width
const int servoMax = 600; // Maximum pulse width

// Brake State
bool brakeAppliedOld = false; // Holds current posisition until new command is given, may need to change

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685 Servo Driver...");
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz (typical for servos)

  // Initialize servos to the release position
  setServoPosition(brakeServo1Channel, servo1ReleasePosition);
  setServoPosition(brakeServo2Channel, servo2ReleasePosition);

  Serial.println("Servo Brake System Initialized.");
}

void loop() {
  if(Serial.available() > 0) { // check if the brake state has changed
    bool brakeApplied = Serial.read();
    if(brakeApplied == brakeAppliedOld) return;
    brakeAppliedOld = brakeApplied;

    if (brakeApplied) { // Brake applied if button is pressed or encoder count exceeds threshold
      // Serial.println("Brake Applied.");
      setServoPosition(brakeServo1Channel, servo1BrakePosition); // Apply brake with Servo 1
      setServoPosition(brakeServo2Channel, servo1BrakePosition); // Apply brake with Servo 2
      // Turn on lights
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);  // Turn on lights

    } else {
      // Serial.println("Brake Released.");
      setServoPosition(brakeServo1Channel, servo1ReleasePosition); // Release brake with Servo 1
      setServoPosition(brakeServo2Channel, servo1ReleasePosition); // Release brake with Servo 2

      digitalWrite(BRAKE_LIGHT_PIN, LOW);   // Turn off lights
    }
  }

}

// Function to set servo position using PCA9685
void setServoPosition(int channel, int angle) {
  int pulseLength = map(angle, 0, 180, servoMin, servoMax);
  pwm.setPWM(channel, 0, pulseLength);
}