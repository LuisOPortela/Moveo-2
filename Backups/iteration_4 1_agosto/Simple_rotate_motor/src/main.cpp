#include <Arduino.h>

const int pulsePin = A0;    // PUL+ pin
const int dirPin = A1;      // DIR+ pin

void setup() {
  // Initialize the pulse and direction pins as outputs
  pinMode(pulsePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // Set the direction to clockwise (you can change this as needed)
  digitalWrite(dirPin, HIGH)
;
  // Send 100 steps
  for (int i = 0; i < 100; i++) {
    // Generate a pulse
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(500);  // Delay for 500 microseconds (50 steps per second)
    //delay(2);
    digitalWrite(pulsePin, LOW);
    //delay(2);
    delayMicroseconds(500);  // Delay for 500 microseconds
  }



  // Pause for a moment before repeating
  //delay(1000);
}