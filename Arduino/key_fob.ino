#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();
const int buzzerPin = 3; // Pin connected to the buzzer
long long lastTime = 0;
long long waitTime = 50;

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0); // Pin D2 by default, for whatever reasons does not work with other pins
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
  lastTime = millis();
}

void loop() {
  long long currentTime = millis();
  // if (currentTime - lastTime >= waitTime && mySwitch.available()) {
  if (mySwitch.available()) {
    long receivedValue = mySwitch.getReceivedValue(); // Get received value

    // Serial.print("Received signal: ");
    // Serial.println(receivedValue);

    // If a signal is received, activate the buzzer
    if (receivedValue != 0) { // Check if the signal is valid
      digitalWrite(buzzerPin, HIGH); // Turn on buzzer
    }

    lastTime = currentTime;
    mySwitch.resetAvailable(); // Reset the receiver for the next signal
  }
  else if(currentTime - lastTime >= waitTime) {
    // Serial.println("Not Found");
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
  }
  
}
