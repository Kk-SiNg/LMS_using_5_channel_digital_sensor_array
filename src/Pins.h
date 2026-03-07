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

// === ENCODERS ===
#define ENCODER_L_A 19      //19
#define ENCODER_L_B 21      //21
#define ENCODER_R_A 39      //22
#define ENCODER_R_B 36      //23

// === MOTOR CONTROL (TB6612FNG) - FIXED ===
#define MOTOR_L_AIN1 17
#define MOTOR_L_AIN2 16   
#define MOTOR_R_BIN1 18   
#define MOTOR_R_BIN2 5

#define MOTOR_L_PWMA 4
#define MOTOR_R_PWMB 22     //15

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