#include <Arduino.h>
#include <AccelStepper.h>
#include "header.h"


//----------------------------------------------------------------
//                 Definitions, Globals and Macros
//     -  Pins
//     -  Serial
//     -  Motor
//
//----------------------------------------------------------------

//12800 pulses per revolution *10 gear ratio
moveoJoint joint_1(128000,1,A0,A1);


const byte maxLength = 10;
char inputBuffer[maxLength];
byte bufferIndex = 0;

//Heartbeat variables
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // Timeout in milliseconds (e.g., 1000 ms = 1 second)


//void decode();

int speed2steps(float speed, int pulsesRev);

//----------------------------------------------------------------
//                         Setup
//     - Pins
//     - Serial
//     - Motor
//
//----------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  
  joint_1.motor.setMaxSpeed(32000);
  joint_1.motor.setSpeed(0);
  
  Serial.begin(9600);

}


//----------------------------------------------------------------
//                         Loop
//     
//     - Read Serial Port
//     - Decode Message
//     - Update values
//
//----------------------------------------------------------------
void loop() 
{
  bool commandReceived = false;
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    
    // If newline is received, process the buffer
    if (receivedChar == '\n') {
      inputBuffer[bufferIndex] = '\0';  // Null-terminate the string
      float speed = atof(inputBuffer);    // Convert buffer to integer
      int steps_sec = speed2steps(speed,joint_1.pulseRev);
      
      // Set the speed of the stepper motor
      joint_1.motor.setSpeed(steps_sec); 	//The desired constant speed in steps per second. Positive is clockwise. Speeds of more than 1000 steps per second are unreliable
      Serial.println(String("Steps per second:")+steps_sec);
      // Reset buffer index for next input
      bufferIndex = 0;
      lastCommandTime = millis(); // Update last command time
      commandReceived = true;    
    } else {
      // Add received character to buffer if it does not exceed maxLength
      if (bufferIndex < maxLength - 1) {
        inputBuffer[bufferIndex++] = receivedChar;
      }
    }
  }
  // Check for timeout
  if (!commandReceived && (millis() - lastCommandTime > commandTimeout)) {
    joint_1.motor.setSpeed(0); // Set speed to 0 if timeout
  }  
  // Run the motor at the set speed
  joint_1.motor.runSpeed();
}


// Convert speed in rads/s to steps per second
int speed2steps(float speed, int pulsesRev)
{
return int(speed*float(pulsesRev)/(2*PI)); //check
}



//void decode()
