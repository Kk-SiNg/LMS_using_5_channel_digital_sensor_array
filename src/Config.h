/*
 * Config.h
 * Central configuration for robot physical dimensions,
 * derived kinematics, speed defaults, and PID gains.
 */

#pragma once
#include <cmath>

// =====================================================================
//  ROBOT PHYSICAL DIMENSIONS
// =====================================================================
constexpr float ENCODER_CPR        = 400.0f;   // Counts Per Revolution
constexpr float WHEEL_DIAMETER_MM  = 44.0f;    // mm
constexpr float WHEEL_RADIUS_MM    = WHEEL_DIAMETER_MM / 2.0f;
constexpr float WHEEL_BASE_MM      = 118.0f;   // mm, center-to-center of drive wheels
constexpr float SENSOR_OFFSET_MM   = 119.0f;   // mm, sensor array to wheel axle
constexpr float CASTOR_OFFSET_MM   = 88.0f;    // mm, castor to wheel axle

// Sensor array physical parameters
constexpr float SENSOR_ARRAY_WIDTH_MM = 56.0f;  // 5.6 cm total
constexpr float SENSOR_PITCH_MM       = SENSOR_ARRAY_WIDTH_MM / 7.0f; // 8mm between sensors
constexpr float LINE_WIDTH_MM         = 30.0f;  // 3 cm white line

// =====================================================================
//  DERIVED KINEMATIC CONSTANTS (compile-time)
// =====================================================================
constexpr float WHEEL_CIRCUMFERENCE_MM = (float)(M_PI * WHEEL_DIAMETER_MM);  // ~138.23 mm
constexpr float MM_PER_TICK            = WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR;  // ~0.3456 mm/tick
constexpr float TICKS_PER_MM           = ENCODER_CPR / WHEEL_CIRCUMFERENCE_MM;  // ~2.894 ticks/mm

// Angle per differential tick
constexpr float RAD_PER_DIFF_TICK      = MM_PER_TICK / WHEEL_BASE_MM;

// Ticks for common angles (for reference / fallback)
constexpr float TICKS_FOR_90_CALC  = (M_PI / 2.0f) * WHEEL_BASE_MM / MM_PER_TICK;  // per wheel, pivot turn
constexpr float TICKS_FOR_180_CALC = M_PI * WHEEL_BASE_MM / MM_PER_TICK;

// =====================================================================
//  SPEED DEFAULTS (in mm/s)
// =====================================================================
constexpr float DEFAULT_CRUISE_SPEED_MMS = 200.0f;   // Moderate cruise speed
constexpr float MAX_SPEED_MMS            = 500.0f;    // Absolute upper bound
constexpr float TURN_SPEED_MMS           = 150.0f;    // Speed during arc turns
constexpr float MIN_SPEED_MMS            = 50.0f;     // Minimum meaningful speed

// PWM limits
constexpr int   PWM_MAX = 255;
constexpr int   PWM_MIN = 0;

// =====================================================================
//  VELOCITY PID DEFAULTS (inner loop, per wheel)
//  Error unit: ticks/interval
// =====================================================================
constexpr float VEL_KP_DEFAULT = 2.0f;    // moderate P for correction
constexpr float VEL_KI_DEFAULT = 0.1f;    // low I to avoid windup
constexpr float VEL_KD_DEFAULT = 0.02f;   // small D for damping

// =====================================================================
//  STEERING PID DEFAULTS (outer loop)
//  Error unit: raw sensor position (0-7000, center at 3500)
//  These are your currently-tuned values!
// =====================================================================
constexpr float STEER_KP_DEFAULT = 0.022f;
constexpr float STEER_KI_DEFAULT = 0.0003f;
constexpr float STEER_KD_DEFAULT = 0.024f;
constexpr float STEER_INTEGRAL_MAX = 500000.0f;

// =====================================================================
//  JUNCTION DETECTION
// =====================================================================
// Distance (mm) that outer-sensor "branch" pattern must persist to confirm
// a real junction vs. a PID-corrected drift artifact.
// Real branch (30mm line) persists for >>15mm; drift corrects in <5mm.
constexpr float JUNCTION_CONFIRM_MM_DEFAULT = 15.0f;

// Minimum distance (mm) between junctions to prevent double-counting
constexpr float JUNCTION_MIN_SPACING_MM = 40.0f;

// Dead-end confirmation time (ms, all sensors off)
constexpr unsigned long DEAD_END_CONFIRM_MS = 150;

// =====================================================================
//  CONTROL LOOP TIMING
// =====================================================================
constexpr unsigned long CONTROL_INTERVAL_US = 1000;  // 1ms → 1 kHz inner loop
constexpr unsigned long STEER_INTERVAL_US   = 2000;  // 2ms → 500 Hz outer loop

// =====================================================================
//  PATH STORAGE
// =====================================================================
#define MAX_PATH_LENGTH 100

// =====================================================================
//  HEADING TURN TARGETS  (radians)
// =====================================================================
constexpr float TURN_90_RAD  = (float)(M_PI / 2.0);
constexpr float TURN_180_RAD = (float)(M_PI);
