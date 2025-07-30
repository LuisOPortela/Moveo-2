#include <Arduino.h>
#include <AccelStepper.h>
#include "header.h"
#include "config.h"

//----------------------------------------------------------------
//                 Definitions, Globals and Macros
//     -  Pins
//     -  Serial
//     -  Motor
//
//----------------------------------------------------------------



const byte maxLength = 100;
char inputBuffer[maxLength];
byte bufferIndex = 0;

//Heartbeat variables
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 500; // Timeout in milliseconds (e.g., 1000 ms = 1 second)

int change_motor_speed(int jointIndex, float speed);
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
  
  Serial.begin(BAUD_RATE);

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
      // Parse full joint command string like "J0:0.12,J1:-0.05,..."
      char* token = strtok(inputBuffer, ",");
      
      while (token != NULL) {
        if (token[0] == 'J') {
          int jointIndex = token[1] - '0';  // example: '3' - '0' = 51 - 48 = 3 → jointIndex = 3
          char* valuePtr = strchr(token, ':');
          if (valuePtr) {
            float speed = atof(valuePtr + 1);
            
            int steps_sec=change_motor_speed(jointIndex, speed);
            
            Serial.print("Joint ");
            Serial.print(jointIndex);
            Serial.print(" steps/sec: ");
            Serial.println(steps_sec);
          }
        }
        token = strtok(NULL, ",");
      }
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

  // Heartbeat check
  if (!commandReceived && (millis() - lastCommandTime > commandTimeout)) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      joints[i].motor.runSpeed();
    }   
  }
  // Run the motor at the set speed
  for (int i = 0; i < NUM_JOINTS; ++i) {
    joints[i].motor.runSpeed();
  }
}


// Convert speed in rads/s to steps per second
int speed2steps(float speed, int pulsesRev)
{
return int(speed*float(pulsesRev)/(2*PI)); //check
}

int change_motor_speed(int jointIndex, float speed)
{
  for(int i = 0; i < NUM_JOINTS; ++i) {
    if (joints[i].id != jointIndex) {
      continue; 
    }
    int steps_sec = speed2steps(speed, joints[i].pulseRev);
    joints[i].motor.setSpeed(steps_sec); // Stop other motors
    return steps_sec;
  }

return -1; // Return the steps/sec value for the joint
}