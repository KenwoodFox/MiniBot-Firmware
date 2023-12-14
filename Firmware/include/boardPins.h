#include <Arduino.h>

#ifndef _BOARD_PINS_H
#define _BOARD_PINS_H

// RGB LED
#define NEO_PIN 38
#define BRIGHTNESS_PIN A0

// Motor controllers
#define MotorPWM_A 4 // port motor
#define MotorPWM_B 5 // star motor
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

// Buttons
#define UB1_PIN 6 // As defined by Prof
#define UB2_PIN 7

// Encoders
#define PORT_ENC 2
#define STAR_ENC 3

// Ultrasonic
#define ECHO_PIN 11
#define TRIG_PIN 12

// Servos
#define SERVO1_PIN 51

// Light sensors
#define LEFT_LINEFOLLOWER 0
#define CNTR_LINEFOLLOWER 1
#define RIGH_LINEFOLLOWER 2

// Steppers
#define STEPA_1 41
#define STEPA_2 43
#define STEPA_3 45
#define STEPA_4 47

#endif
