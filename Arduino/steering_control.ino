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
const int enA = 9;  	// PWM control for motor speed
const int in1 = 8;  	// Motor driver input 1
const int in2 = 7;  	// Motor driver input 2

// Encoder pins:
const int EncoderA = 2; // Encoder channel A (interrupt pin)
const int EncoderB = 3; // Encoder channel B

// Constants
const int MAX_PWM = 255;    	 
const int MIN_PWM = 50;     	 
const int pulsesPerRevolution = 20;  // Adjust this based on your encoder



// Encoder variables:
volatile long encoderCount = 0;  	// Pulse counter
volatile int encoderDirection = 0;   // 1 = forward, -1 = reverse

// serial read in
int serial_read_pwm = 0;

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
  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderISR, RISING);

  lastTime = millis();
}

void loop() {
  /// TODO: control based on angle
  while(Serial.available() > 0) {
    // Serial.println("recieved");
    serial_read_pwm = Serial.read(); // 1 byte signed
  }

  run_motor(serial_read_pwm);
}

void run_motor(signed char serial_in) {
	// Serial.print("serial_in: "); Serial.println(serial_in);

    if (serial_in > 0 && encoderCount < 1500) {
      int pwm_i = constrain(serial_in, MIN_PWM, MAX_PWM);
      forwardMotor(pwm_i);
    } else if (serial_in < 0 && encoderCount > -1500) {
      int pwm_i = constrain(-serial_in, MIN_PWM, MAX_PWM);
      reverseMotor(pwm_i);
    } else {
      stopMotor();
    }
    
}

// Function to drive the motor forward
void forwardMotor(int pwm) {
//   Serial.println("Motor set to forward direction.");
  encoderDirection = 0;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  analogWrite(enA, pwm);
}

// Function to drive the motor in reverse
void reverseMotor(int pwm) {
//   Serial.println("Motor set to reverse direction.");
  encoderDirection = 0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  analogWrite(enA, pwm);
}

// Function to brake (stop) the motor
void stopMotor() {
  // Serial.println("Stoping motor...");
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