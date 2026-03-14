/*
 * Odometry.h
 * Differential-drive encoder odometry.
 * Provides pose (x, y, θ) and segment distance in physical units (mm, rad).
 */

#pragma once
#include <ESP32Encoder.h>
#include "Config.h"

class Odometry {
public:
    Odometry();

    // Call once in setup() to bind encoder references
    void begin(ESP32Encoder* leftEnc, ESP32Encoder* rightEnc);

    // Call every control cycle to update pose from encoder deltas
    void update();

    // --- Pose getters ---
    float getX() const        { return x; }
    float getY() const        { return y; }
    float getHeading() const  { return theta; }   // radians

    // --- Segment tracking ---
    float getSegmentMM() const   { return segmentDist; }
    void  resetSegment()         { segmentDist = 0.0f; }

    // --- Velocity (mm/s) ---
    float getLinearVelocity() const  { return linearVel; }
    float getAngularVelocity() const { return angularVel; }

    // --- Per-wheel velocity (ticks per interval) ---
    float getLeftTickRate() const  { return leftTickRate; }
    float getRightTickRate() const { return rightTickRate; }

    // --- Reset everything ---
    void reset();

    // --- Raw encoder access (for Motors velocity PID) ---
    long getLeftDelta() const  { return lastLeftDelta; }
    long getRightDelta() const { return lastRightDelta; }

private:
    ESP32Encoder* pLeftEnc;
    ESP32Encoder* pRightEnc;

    long prevLeftCount;
    long prevRightCount;
    long lastLeftDelta;
    long lastRightDelta;

    float x, y, theta;          // Pose in mm and radians
    float segmentDist;           // Distance since last resetSegment()

    float linearVel;             // mm/s
    float angularVel;            // rad/s
    float leftTickRate;          // ticks per interval (for velocity PID)
    float rightTickRate;

    unsigned long lastUpdateUs;  // micros() at last update
};
