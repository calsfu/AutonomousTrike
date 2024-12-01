#include <Servo.h>

Servo brakeServo; // Create a servo object
const int buttonPin = 2; // Pin for the button
const int servoPin = 9; // Pin for the servo motor

// Define servo positions
const int releasePosition = 0;  // Position to release the brake
const int brakePosition = 90;  // Position to apply the brake

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Configure button pin with internal pull-up resistor
  brakeServo.attach(servoPin);     // Attach the servo motor to the specified pin

  // Initialize the servo in the release position
  brakeServo.write(releasePosition);
  Serial.begin(9600);
  Serial.println("Servo Brake System Initialized.");
}

void loop() {
  int buttonState = digitalRead(buttonPin); // Read the button state

  if (buttonState == LOW) { // Button pressed (active LOW)
    Serial.println("Brake Applied.");
    brakeServo.write(brakePosition); // Move the servo to apply the brake
    delay(500); // Hold the brake for a short duration (adjust as needed)
  } else {
    Serial.println("Brake Released.");
    brakeServo.write(releasePosition); // Move the servo to release the brake
  }

  delay(100); // Small delay to debounce button input
}
