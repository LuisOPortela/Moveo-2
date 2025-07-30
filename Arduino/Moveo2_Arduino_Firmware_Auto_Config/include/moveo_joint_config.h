// Auto-generated file from ros2_control.xacro
#pragma once
#include "header.h"

#define BAUD_RATE 9600
// Serial device: /dev/ttyACM0
// I2C bus: /dev/i2c-1
#define SERIAL_TIMEOUT_MS 1000

#define NUM_JOINTS 2

moveoJoint joints[NUM_JOINTS] = {
  moveoJoint(32000, 1, A0, A1), // Joint_1
  moveoJoint(6969, 1, A2, A3), // Joint_2
};



// // config.h

// #ifndef CONFIG_H
// #define CONFIG_H


// #define NUM_JOINTS 2

// moveoJoint joints[NUM_JOINTS] = {
//   joint_1(1600, 1, 1, 1), // Joint_1
//   moveoJoint(99999, 1, 1, 1), // Joint_2
// };

// // Serial settings
// #define SERIAL_BAUDRATE 9600

// #define NR_OF_JOINTS 1

// // Joint 1 configuration
// //3200 pulses per revolution *10 gear ratio
// #define JOINT1_PULSES_PER_REV 32000
// #define JOINT1_DIRECTION 1
// #define JOINT1_PIN_PULSE A0
// #define JOINT1_PIN_DIR A1

// #define JOINT1_MAX_SPEED 32000
// #define JOINT1_INITIAL_SPEED 0

// // Joint 2 configuration

// #define JOINT2_PULSES_PER_REV 32000
// #define JOINT2_DIRECTION 1
// #define JOINT2_PIN_PULSE A2
// #define JOINT2_PIN_DIR A3       

// #define JOINT2_MAX_SPEED 32000
// #define JOINT2_INITIAL_SPEED 0

// #endif