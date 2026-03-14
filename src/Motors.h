/*
 * Motors.h
 * Motor control with cascade PID architecture.
 * Inner loop: per-wheel velocity PID (QuickPID).
 * Outer loop (steering) is handled in main.cpp.
 */

#pragma once
#include "Pins.h"
#include "Config.h"
#include <ESP32Encoder.h>
#include <QuickPID.h>

class Sensors;  // forward decl for legacy compatibility

// WiFi-tunable speed/motor parameters
extern float cruiseSpeedMMS;
extern float maxSpeedMMS;
extern float turnSpeedMMS;

class Motors {
public:
    Motors();
    void setup();

    // --- Low-level PWM control (calibration / direct override) ---
    void setSpeeds(int leftPWM, int rightPWM);
    void stopBrake();

    // --- Velocity PID (inner loop) ---
    // Set target wheel velocities in mm/s.  Positive = forward.
    void setTargetVelocities(float leftMMS, float rightMMS);
    // Call every control cycle (~1 kHz).  Reads encoder deltas, computes PID, writes PWM.
    void updateVelocityPID();
    // Tune velocity PID gains live
    void setVelPIDGains(float kp, float ki, float kd);

    // --- Utility ---
    void moveForward(int ticks);  // blocking, for calibration only
    void rotate();                // open-loop spin for sensor calibration

    // --- Encoder access (for Odometry) ---
    ESP32Encoder* getLeftEncoder()  { return &leftEncoder; }
    ESP32Encoder* getRightEncoder() { return &rightEncoder; }
    long getLeftCount()   { return leftEncoder.getCount(); }
    long getRightCount()  { return rightEncoder.getCount(); }
    long getAverageCount(){ return (leftEncoder.getCount() + rightEncoder.getCount()) / 2; }
    void clearEncoders();

    // --- WiFi tuning helpers ---
    static void updateSpeeds(float cruise, float turn, float maxSpd);

    // --- Velocity PID state (for telemetry) ---
    float getLeftTargetMMS()  const { return leftTargetMMS; }
    float getRightTargetMMS() const { return rightTargetMMS; }
    float getLeftPWMOut()     const { return leftPWMOutput; }
    float getRightPWMOut()    const { return rightPWMOutput; }

private:
    // PWM channels
    const int pwm_channel_left  = 0;
    const int pwm_channel_right = 1;
    const int pwm_frequency  = 5000;
    const int pwm_resolution = 8;

    // Encoders
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;

    // Velocity PID state
    float leftTargetMMS,  rightTargetMMS;
    float leftTargetTicks, rightTargetTicks;   // target in ticks/interval
    float leftMeasuredTicks, rightMeasuredTicks;
    float leftPWMOutput, rightPWMOutput;

    // Per-wheel QuickPID controllers
    QuickPID leftVelPID;
    QuickPID rightVelPID;

    // Previous encoder counts (for delta calculation)
    long prevLeftCount, prevRightCount;
    unsigned long lastVelUpdateUs;

    // Convert mm/s to ticks per control interval
    float mmsToTicksPerInterval(float mms, float dtSec);

    // Apply PWM to hardware
    void applyPWM(int leftPWM, int rightPWM);
};