// Auto-generated file from ros2_control.xacro
#pragma once
#include "header.h"
#include <Arduino.h>

#define BAUD_RATE 9600
// Serial device: /dev/ttyACM0
// I2C bus: /dev/i2c-1
#define SERIAL_TIMEOUT_MS 1000

#define NUM_JOINTS 2

moveoJoint joints[] = {
  moveoJoint(1,32000, 1, A0, A1), // Joint_1
  moveoJoint(2,6969, 1, A2, A3), // Joint_2
};
