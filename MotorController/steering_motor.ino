// TODO: create ros node and control motor with ros node

/*
  Modified code for your motor pinouts with encoder feedback and RPM calculation:

  Motor driver pins:
    enA  - PWM control for motor speed (pin 9)
    in1  - Motor driver input 1 (pin 8)
    in2  - Motor driver input 2 (pin 7)

  Encoder pins:
    EncoderA - Encoder channel A (pin 2, interrupt pin)
    EncoderB - Encoder channel B (pin 3)
*/

// Motor driver pins:
const int enA = 9;      // PWM control for motor speed
const int in1 = 8;      // Motor driver input 1
const int in2 = 7;      // Motor driver input 2

// Encoder pins:
const int EncoderA = 2; // Encoder channel A (interrupt pin)
const int EncoderB = 3; // Encoder channel B

// Constants
const int MAX_PWM = 255;         
const int MIN_PWM = 50;          
const int pulsesPerRevolution = 20;  // Adjust this based on your encoder

// Encoder variables:
volatile long encoderCount = 0;      // Pulse counter
volatile int encoderDirection = 0;   // 1 = forward, -1 = reverse

// Time tracking for RPM calculation
unsigned long lastTime = 0;
float rpm = 0;  // Stores calculated RPM

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(EncoderA, INPUT_PULLUP);
  pinMode(EncoderB, INPUT_PULLUP);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderISR, CHANGE);

  lastTime = millis();
}

void loop() {

  int pwm;
  brakeMotor();
  delay(50);

  // Test forward rotation
  for (pwm = MIN_PWM; pwm <= MAX_PWM; pwm += 80) {
    forwardMotor(pwm);
    delay(2000);
    calc_rpm();
  }

  // Test reverse rotation
  for (pwm = MIN_PWM; pwm <= MAX_PWM; pwm += 80) {
    reverseMotor(pwm);
    delay(2000);
    calc_rpm();
  }
  
  brakeMotor();
  delay(500);
  calc_rpm();
}

// Function to drive the motor forward
void forwardMotor(int pwm) {
  Serial.println("Motor set to forward direction.");
  encoderDirection = 0; 

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  analogWrite(enA, pwm);
}

// Function to drive the motor in reverse
void reverseMotor(int pwm) {
  Serial.println("Motor set to reverse direction.");
  encoderDirection = 0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  analogWrite(enA, pwm);
}

// Function to brake (stop) the motor
void brakeMotor() {
  Serial.println("Braking motor...");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

// Function to print encoder direction and RPM
void print_direction() {
//  Serial.print("Encoder A: "); Serial.print(digitalRead(EncoderA));
//  Serial.print(" | Encoder B: "); Serial.print(digitalRead(EncoderB));
//  Serial.print(" | Count: "); Serial.print(encoderCount);
  Serial.print(" | Direction: "); Serial.print(encoderDirection);
  Serial.print(" | RPM: "); Serial.println(rpm);
}

void calc_rpm() {
   unsigned long currentTime = millis();
  float timeInterval = (currentTime - lastTime) / 1000.0; // Convert to seconds

  // calc revolutions per minute 
  if (timeInterval >= 0.5) {  // Update RPM every 0.5 sec
    rpm = (encoderCount * 60.0) / (pulsesPerRevolution * timeInterval);
    encoderCount = 0;  
    lastTime = currentTime; 
    
    print_direction();
  }
}

// Interrupt Service Routine (ISR) for Encoder
// when a!=b, forward, count--;
// when a==b, reverse, count++;
void encoderISR() {
  if (digitalRead(EncoderA) == digitalRead(EncoderB)) {
      encoderCount--;
      encoderDirection = -1; // Reverse
  } else {
      encoderCount++;
      encoderDirection = 1;  // Forward
  }
}
