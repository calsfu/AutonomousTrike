#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();
const int buzzerPin = 3; // Pin connected to the buzzer

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0); // Pin D2 by default, for whatever reasons does not work with other pins
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
}

void loop() {
  if (mySwitch.available()) {
    long receivedValue = mySwitch.getReceivedValue(); // Get received value

    Serial.print("Received signal: ");
    Serial.println(receivedValue);

    // If a signal is received, activate the buzzer
    if (receivedValue != 0) { // Check if the signal is valid
      digitalWrite(buzzerPin, HIGH); // Turn on buzzer
      delay(500);                    // Buzzer on for 500ms (adjust as needed)
      digitalWrite(buzzerPin, LOW);  // Turn off buzzer
    }

    mySwitch.resetAvailable(); // Reset the receiver for the next signal
  }
}
