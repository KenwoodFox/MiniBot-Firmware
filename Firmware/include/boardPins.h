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

// Encoders
#define STAR_ENC 2
#define PORT_ENC 3

#endif
