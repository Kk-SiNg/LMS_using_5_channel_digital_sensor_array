/*
 * Pins.h
 * Hardware pin definitions - DIGITAL MODE
 * Target: ESP32 38-Pin DevKit (NodeMCU-32S)
 */

#pragma once
#include <cstdint>

// =========================================================
//  LEFT RAIL: SENSOR ARRAY
//  Physically, Pin 36 is at the TOP (near USB/Antenna)
//  Physically, Pin 26 is further DOWN.
// =========================================================

// Mapping assumes sensors are connected sequentially to the Left Rail

// === SENSOR PINOUT ===
#define SENSOR_PIN_1  32  // Rightmost Sensor
#define SENSOR_PIN_2  33
#define SENSOR_PIN_3  25
#define SENSOR_PIN_4  26
#define SENSOR_PIN_5  27
#define SENSOR_PIN_6  14
#define SENSOR_PIN_7  12
#define SENSOR_PIN_8  13  // Leftmost Sensor
const uint8_t SensorCount = 8;

// Digital mode settings
#define LINE_THRESHOLD 1              // Digital: 1 = line detected
#define MIN_DETECTION_RATIO 0.5
#define CALIBRATION_TIME_MS 1000

// =========================================================
//  RIGHT RAIL (USB Top): MOTORS & ENCODERS
//  Pins are listed in physical order (Top to Bottom)
//  Avoids GPIO 6-11 (Flash) and GPIO 1 (TX0)/3 (RX0)
// =========================================================

// === ENCODERS ===
// GPIO 23, 22, 21 are standard IO. 
// GPIO 5 is a strapping pin (must be HIGH during boot), but standard
// encoders usually leave this floating or high-Z enough to be safe.

// === ENCODERS ===
#define ENCODER_L_A 34      //19
#define ENCODER_L_B 35      //21
#define ENCODER_R_A 36      //22
#define ENCODER_R_B 39      //23

// === MOTOR CONTROL (TB6612FNG) - FIXED ===
#define MOTOR_L_AIN1 17   
#define MOTOR_L_AIN2 16   
#define MOTOR_R_BIN1 18   
#define MOTOR_R_BIN2 5

#define MOTOR_L_PWMA 4
#define MOTOR_R_PWMB 15

#define MOTOR_STBY   2    // Standby Pin (will set HIGH in setup)

// === RGB LED ===
// Common Cathode (-) recommended to keep Pins 12/13 LOW during boot.
// #define RGB_PIN_R  36  // Safe GPIO
// #define RGB_PIN_G  39  // MTDI (Strapping: Must NOT be pulled HIGH at boot)
// #define RGB_PIN_B  34  // Safe GPIO

// === USER INTERFACE ===
#define ONBOARD_LED 2   // Blue LED on DevKit
#define USER_BUTTON 0   // BOOT Button (Active LOW)

// === WIFI CONFIGURATION ===
#define WIFI_SSID "KKS's phone"
#define WIFI_PASS "kvsandkks"
#define TELNET_PORT 23