/*  
  L298N Motor Demonstration
  L298N-Motor-Demo.ino
  Demonstrates functions of L298N Motor Controller
  
  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/
  
 
// Motor A
 
int enA = 9;
int in1 = 8;
int in2 = 7;
int EncoderA = 2;
int EncoderB = 3;

volatile int posi = 0;
int steeringValue = 0;

void setup()
{
 
  // Set all the motor control pins to outputs
  Serial.begin(115200);
 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(EncoderA),readEncoder,RISING);
 
}
 
void demoOne()
{
  // This function will run the motors in both directions at a fixed speed
 
  // Turn on motor A
  Serial.println("DEMO ONE START");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 
  // Set speed to 200 out of possible range 0~255
  
  analogWrite(enA, 200);
  
  delay(2000);

  // motor off
  Serial.println("MOTOR STOPS");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW); 

  delay(200);

 
  // Now change motor directions
  Serial.println("MOTOR REVERSE");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
 
  delay(2000);
 
  // Now turn off motors
  Serial.println("MOTOR OFF");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
}

// test to see if its working with Orin connection
void demoTwo() 
{
  Serial.println("Staring demo for Orin connection");
  if(Serial.available() > 0) {
    steeringValue = Serial.read();
  }
  int motorSpeed = steeringValue * 255;

  // if(steeringValue > 0.05) {
  //   // steer right
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, HIGH);
  //   analogWrite(enA, motorSpeed);

  // }
  // else if(steeringValue < -0.05) {
  //   // Steer left
  //   digitalWrite(in1, HIGH);
  //   digitalWrite(in2, LOW);
  //   analogWrite(enA, motorSpeed);
  // }
  // else {
  //   // Stop motor
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, LOW);
  // }

  if(steeringValue == 1) {
    // steer right
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, motorSpeed);
    delay(1000);
  }
  else if(steeringValue == 2) {
    // Steer left
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, motorSpeed);
    delay(1000);
  }
  steeringValue = 0;

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  Serial.flush();

}


void testReversible() {
  
}

void testVariableSpeed() {
  
}

void testMaxSpeed() {
  
}

void testReverseSpeed() {
  
}

void testEncoderFeedback() {
  
}
/*
 
void demoTwo()
 
{
 
  // This function will run the motors across the range of possible speeds
  // Note that maximum speed is determined by the motor itself and the operating voltage
 
  // Turn on motors
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
 
  // Accelerate from zero to maximum speed
 
  for (int i = 0; i < 256; i++)
 
  {
 
    analogWrite(enA, i);
    analogWrite(enB, i);
 
    delay(20);
 
  } 
 
  // Decelerate from maximum speed to zero
 
  for (int i = 255; i >= 0; --i)
 
  {
 
    analogWrite(enA, i);
    analogWrite(enB, i);
 
    delay(20);
 
  } 
 
  // Now turn off motors
 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
 
}
*/
void readEncoder() {
  int b = digitalRead(EncoderB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
  //Serial.println(posi);
}

 
void loop()
 
{
 
  demoTwo();
  // delay(1000);
  //testReversible(); //Test if the motor can go both directions
  //delay(1000);
  
  //testVariableSpeed(); //Test how the motor responds to incrementing the speed
  //delay(1000);
  
  //testMaxSpeed(); //Test how the motor runs at max speed
  //delay(1000);
  
  //testReverseSpeed(); //Find and test the appropriate & fastest time it takes to switch speeds
  //delay(1000);

  //testEncoderFeedback(); //Test if the motor speed can be controlled by the encoer readings (ie spin until position x, then reverse direction)
  //delay(1000);

  
}