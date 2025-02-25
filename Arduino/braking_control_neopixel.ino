#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <RCSwitch.h>
#include <Adafruit_NeoPixel.h>

// LED strip configuration for addressable LEDs (e.g., WS2812B)
//#define LED_PIN 5         // Connect Din to digital pin 5
//#define NUM_LEDS 30       // Adjust to the number of LEDs on your strip
//#define BRIGHTNESS 100    // Brightness (0-255)
//#define LED_TYPE WS2812B  // Change if your LED strip type differs
//#define COLOR_ORDER GRB   // Most strips use GRB
#define PIN        5
#define NUM_LEDS 50
//CRGB leds[NUM_LEDS];

// Create an instance of the PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
RCSwitch mySwitch = RCSwitch();
Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// Servo channel assignments
const int brakeServo1Channel = 0; // Channel for Servo 1
const int brakeServo2Channel = 1; // Channel for Servo 2

// Encoder pin
const int encoderPin = 7; // Pin connected to the encoder's signal output

// Button pin
const int buttonPin = 4; // Pin connected to the brake button

// Buzzer pin
const int buzzerPin = 3;

// Define servo positions
const int releasePosition = 0;  // Position to release the brake
const int brakePosition = 30;   // Position to apply the brake

// Define PCA9685 pulse width range
const int servoMin = 150; // Minimum pulse width
const int servoMax = 600; // Maximum pulse width

// Define variables for the encoder state
volatile int encoderState = LOW; // The state of the encoder
volatile int encoderCount = 0;   // Encoder pulse count

// RF variables
long receivedValue = 0; // Store received RF signal

bool brakeApplied = false; // Holds current posisition until new command is given, may need to change
int oldButtonState = LOW;

void setup() {
  pinMode(encoderPin, INPUT); // Configure encoder pin as input
  pinMode(buttonPin, INPUT_PULLUP); // Configure button pin as input with pull-up resistor
  pinMode(buzzerPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE); // Attach interrupt to detect encoder signal changes

  Serial.begin(9600);
  Serial.println("Initializing PCA9685 Servo Driver...");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz (typical for servos)
  delay(10);

  Serial.println("Initialized PCA9685 Servo Driver...");

  // Initialize servos to the release position
  setServoPosition(brakeServo1Channel, releasePosition);
  setServoPosition(brakeServo2Channel, releasePosition);

  // Initialize RF Receiver
  mySwitch.enableReceive(0); // Using interrupt 0 (Pin 2 on most boards)
  Serial.println("RF Key Fob System Initialized.");

  Serial.println("Servo Brake System Initialized.");

  // Initialize the LED strip using FastLED
//  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
//  FastLED.setBrightness(BRIGHTNESS);
//  FastLED.clear();
//  FastLED.show();
  strip.begin();
  strip.show();
  
  Serial.println("LED Initialized");
}

void loop() {
  int buttonState = digitalRead(buttonPin); // Read the button state

  if(buttonState == LOW) // button override
  {
    brakeApplied = true;
    oldButtonState = LOW;
  }
  else if(Serial.available() > 0) { // check for nano input
    brakeApplied = Serial.read();
  }

  // Logic to apply brake based on button press or encoder count
  if (brakeApplied) { // Brake applied if button is pressed or encoder count exceeds threshold
//    Serial.println("Brake Applied.");
    setServoPosition(brakeServo1Channel, brakePosition); // Apply brake with Servo 1
    setServoPosition(brakeServo2Channel, brakePosition); // Apply brake with Servo 2
    // Set LED strip to red to indicate braking
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255,0,0));
    }
    
    // push updated LED 
//    FastLED.show();
  } else {
//    Serial.println("Brake Released.");
    setServoPosition(brakeServo1Channel, releasePosition); // Release brake with Servo 1
    setServoPosition(brakeServo2Channel, releasePosition); // Release brake with Servo 2
    strip.clear();
    //FastLED.clear();
  }
    strip.show();

  // RF Key Fob Logic
  if (mySwitch.available()) {
    receivedValue = mySwitch.getReceivedValue(); // Get received value
//    Serial.print("Received RF signal: ");
//    Serial.println(receivedValue);

    // Check if the received value is valid
    if (receivedValue != 0) {
//      Serial.println("Valid Signal! Activating Buzzer...");
      digitalWrite(buzzerPin, HIGH); // Turn on buzzer
      // delay(500);                    // Buzzer on for 500 ms
      digitalWrite(buzzerPin, LOW);   // Turn off buzzer

      // Reset received value to allow repeated activations
      receivedValue = 0;
    }

    mySwitch.resetAvailable(); // Reset RF receiver for the next signal
  }

//  Serial.flush();

  // delay(100); // Small delay to debounce button input
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
