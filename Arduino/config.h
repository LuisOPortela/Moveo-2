// Auto-generated file from ros2_control.xacro
#pragma once
#include "header.h"
#include <Arduino.h>

#define BAUD_RATE 115200
// Serial device: /dev/ttyACM0
// I2C bus: /dev/i2c-1
#define SERIAL_TIMEOUT_MS 1000

#define NUM_JOINTS 2

moveoJoint joints[NUM_JOINTS] = {
  moveoJoint(1,32000, 1, 52, 50), // Joint_1
  moveoJoint(2,16000, 1, 48, 46), // Joint_2
};
