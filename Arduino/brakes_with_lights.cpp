#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RCSwitch.h>
#include <FastLED.h>

// Create an instance of the PCA9685 driver for servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Create an instance of the RCSwitch for RF communication
RCSwitch mySwitch = RCSwitch();

// LED strip configuration for addressable LEDs (e.g., WS2812B)
#define LED_PIN 5         // Connect Din to digital pin 5
#define NUM_LEDS 30       // Adjust to the number of LEDs on your strip
#define BRIGHTNESS 100    // Brightness (0-255)
#define LED_TYPE WS2812B  // Change if your LED strip type differs
#define COLOR_ORDER GRB   // Most strips use GRB
CRGB leds[NUM_LEDS];

// Servo channel assignments
const int brakeServo1Channel = 0; // Channel for Servo 1
const int brakeServo2Channel = 1; // Channel for Servo 2

// Encoder and button pins
const int encoderPin = 7;  // Pin connected to the encoder's signal output
const int buttonPin = 4;   // Pin connected to the brake button

// Buzzer pin for RF signal alert
const int buzzerPin = 3; // Pin connected to the buzzer

// Define servo positions (in degrees)
const int releasePosition = 0;  // Position to release the brake
const int brakePosition = 30;   // Position to apply the brake

// Define PCA9685 pulse width range for servos
const int servoMin = 150; // Minimum pulse length
const int servoMax = 600; // Maximum pulse length

// Variables for encoder state and count
volatile int encoderState = LOW; // The state of the encoder
volatile int encoderCount = 0;   // Encoder pulse count

void setup() {
  // Setup encoder and brake button pins
  pinMode(encoderPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);
  
  Serial.begin(9600);
  Serial.println("Initializing PCA9685 Servo Driver...");
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz is typical for servos
  delay(10);
  
  // Initialize servos to the release position
  setServoPosition(brakeServo1Channel, releasePosition);
  setServoPosition(brakeServo2Channel, releasePosition);
  
  Serial.println("Servo Brake System Initialized.");
  
  // Setup RF receiver and buzzer
  mySwitch.enableReceive(0);  // Default receiver pin is D2
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize the LED strip using FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}

void loop() {
  // Read the brake button state
  int buttonState = digitalRead(buttonPin);
  bool brakeApplied = false;
  
  // Apply brake if the button is pressed or if the encoder count exceeds a threshold
  if (buttonState == LOW || encoderCount > 10) {
    Serial.println("Brake Applied.");
    setServoPosition(brakeServo1Channel, brakePosition);
    setServoPosition(brakeServo2Channel, brakePosition);
    brakeApplied = true;
    
    // Set LED strip to red to indicate braking
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
  } else {
    Serial.println("Brake Released.");
    setServoPosition(brakeServo1Channel, releasePosition);
    setServoPosition(brakeServo2Channel, releasePosition);
    
    // Turn off the LED strip when brakes are released
    FastLED.clear();
  }
  
  // Update the LED strip
  FastLED.show();
  
  // RF Receiver Logic: Trigger buzzer on every valid RF signal
  if (mySwitch.available()) {
    long receivedValue = mySwitch.getReceivedValue();
    Serial.print("Received RF signal: ");
    Serial.println(receivedValue);
    
    if (receivedValue != 0) {
      Serial.println("Valid RF Signal Detected!");
      digitalWrite(buzzerPin, HIGH);
      delay(500);
      digitalWrite(buzzerPin, LOW);
    }
    mySwitch.resetAvailable();
  }
  
  delay(100); // Delay for debounce and processing
}

// Interrupt Service Routine (ISR) for encoder signal changes
void encoderISR() {
  encoderState = digitalRead(encoderPin);
  if (encoderState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Function to set servo position using the PCA9685 driver
void setServoPosition(int channel, int angle) {
  int pulseLength = map(angle, 0, 180, servoMin, servoMax);
  pwm.setPWM(channel, 0, pulseLength);
}
