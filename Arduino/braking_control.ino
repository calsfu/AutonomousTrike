#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastLED.h>

// LED strip configuration for addressable LEDs (e.g., WS2812B)
#define LED_PIN_1 5         // Connect Din to digital pin 5
#define LED_PIN_2 6         // Connect Din to digital pin 6
#define NUM_LEDS 20       // Adjust to the number of LEDs on your strip
#define BRIGHTNESS 100    // Brightness (0-255)
#define LED_TYPE WS2812B  // Change if your LED strip type differs
#define COLOR_ORDER GRB   // Most strips use GRB
CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel assignments
const int brakeServo1Channel = 0; // Channel for Servo 1
const int brakeServo2Channel = 1; // Channel for Servo 2

// Define servo positions
const int releasePosition = 0;  // Position to release the brake
const int brakePosition = 30;   // Position to apply the brake

// Define PCA9685 pulse width range
const int servoMin = 150; // Minimum pulse width
const int servoMax = 600; // Maximum pulse width

// Brake State
bool brakeAppliedOld = false; // Holds current posisition until new command is given, may need to change

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685 Servo Driver...");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz (typical for servos)

  // Initialize servos to the release position
  setServoPosition(brakeServo1Channel, releasePosition);
  setServoPosition(brakeServo2Channel, releasePosition);

  Serial.println("Servo Brake System Initialized.");

  // Initialize the LED strip using FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds1, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds2, NUM_LEDS);

  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  Serial.println("LED Initialized");
}

void loop() {
  if(Serial.available() > 0) { // check if the brake state has changed
    bool brakeApplied = Serial.read();
    if(brakeApplied == brakeAppliedOld) return;
    brakeAppliedOld = brakeApplied;

    if (brakeApplied) { // Brake applied if button is pressed or encoder count exceeds threshold
      // Serial.println("Brake Applied.");
      setServoPosition(brakeServo1Channel, brakePosition); // Apply brake with Servo 1
      setServoPosition(brakeServo2Channel, brakePosition); // Apply brake with Servo 2
      // Set LED strip to red to indicate braking
      fill_solid(leds1, NUM_LEDS, CRGB::Red);
      fill_solid(leds2, NUM_LEDS, CRGB::Red);
    } else {
      // Serial.println("Brake Released.");
      setServoPosition(brakeServo1Channel, releasePosition); // Release brake with Servo 1
      setServoPosition(brakeServo2Channel, releasePosition); // Release brake with Servo 2

      FastLED.clear();
    }
    FastLED.show();
  }

}

// Function to set servo position using PCA9685
void setServoPosition(int channel, int angle) {
  int pulseLength = map(angle, 0, 180, servoMin, servoMax);
  pwm.setPWM(channel, 0, pulseLength);
}