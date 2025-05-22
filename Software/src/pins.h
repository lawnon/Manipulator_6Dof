/*
 * File: pins.h
 * Autor: Copywrite (c) Chukwunonso Bob-Anyeji
 * Description: Holds Pin definition for Ramps 1.4
 *              Shield on an Arduino Mega2560 Controller.
 * Date: 09.06.2024
 */

// Board Information
#define BOARD_INFO_NAME "RAMPS 1.4"

// Servo Pins
#define SERVO1_PIN 11
#define SERVO2_PIN 6
#define SERVO3_PIN 5
#define SERVO4_PIN 4

// Limit Switchs
#define JT1_LIM_PIN 3
#define JT2_LIM_PIN 2
#define JT3_LIM_PIN 14
#define JT4_LIM_PIN 15
#define JT5_LIM_PIN 18
#define JT6_LIM_PIN 19

// Stepper Motor Pins
#define JT1_STP_PIN 40
#define JT1_DIR_PIN 42
#define JT1_ENA_PIN 44

#define JT2_STP_PIN 46
#define JT2_DIR_PIN 48
#define JT2_ENA_PIN 62

#define JT3_STP_PIN 60
#define JT3_DIR_PIN 61
#define JT3_ENA_PIN 56

#define JT4_STP_PIN 54
#define JT4_DIR_PIN 55
#define JT4_ENA_PIN 38

#define JT5_STP_PIN 26
#define JT5_DIR_PIN 28
#define JT5_ENA_PIN 24

#define JT6_STP_PIN 36
#define JT6_DIR_PIN 34
#define JT6_ENA_PIN 30

// Rotary Encoder Pins
#define ENC_INTP_ID 1
#define ENC_CLK_PIN 3
#define ENC_DIR_PIN 47
#define ENC_BTN_PIN 45
