#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel assignments
const int brakeServo1Channel = 0; // Channel for Servo 1
const int brakeServo2Channel = 1; // Channel for Servo 2

// Encoder pin
const int encoderPin = 7; // Pin connected to the encoder's signal output

// Button pin
const int buttonPin = 4; // Pin connected to the brake button

// Define servo positions
const int releasePosition = 0;  // Position to release the brake
const int brakePosition = 30;   // Position to apply the brake

// Define PCA9685 pulse width range
const int servoMin = 150; // Minimum pulse width
const int servoMax = 600; // Maximum pulse width

// Define variables for the encoder state
volatile int encoderState = LOW; // The state of the encoder
volatile int encoderCount = 0;   // Encoder pulse count

void setup() {
  pinMode(encoderPin, INPUT); // Configure encoder pin as input
  pinMode(buttonPin, INPUT_PULLUP); // Configure button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE); // Attach interrupt to detect encoder signal changes

  Serial.begin(9600);
  Serial.println("Initializing PCA9685 Servo Driver...");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz (typical for servos)
  delay(10);

  // Initialize servos to the release position
  setServoPosition(brakeServo1Channel, releasePosition);
  setServoPosition(brakeServo2Channel, releasePosition);

  Serial.println("Servo Brake System Initialized.");
}

void loop() {
  int buttonState = digitalRead(buttonPin); // Read the button state

  // Logic to apply brake based on button press or encoder count
  if (buttonState == LOW || encoderCount > 10) { // Brake applied if button is pressed or encoder count exceeds threshold
    Serial.println("Brake Applied.");
    setServoPosition(brakeServo1Channel, brakePosition); // Apply brake with Servo 1
    setServoPosition(brakeServo2Channel, brakePosition); // Apply brake with Servo 2
  } else {
    Serial.println("Brake Released.");
    setServoPosition(brakeServo1Channel, releasePosition); // Release brake with Servo 1
    setServoPosition(brakeServo2Channel, releasePosition); // Release brake with Servo 2
  }

  delay(100); // Small delay to debounce button input
}

// Interrupt Service Routine (ISR) for encoder signal change
void encoderISR() {
  encoderState = digitalRead(encoderPin); // Read the current state of the encoder
  if (encoderState == HIGH) {
    encoderCount++; // Increment the encoder count when signal goes HIGH
  } else {
    encoderCount--; // Decrement the encoder count when signal goes LOW
  }
}

// Function to set servo position using PCA9685
void setServoPosition(int channel, int angle) {
  int pulseLength = map(angle, 0, 180, servoMin, servoMax);
  pwm.setPWM(channel, 0, pulseLength);
}
