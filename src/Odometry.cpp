/*
 * Odometry.cpp
 * Differential-drive kinematics using encoder feedback.
 */

#include "Odometry.h"
#include <Arduino.h>
#include <math.h>

Odometry::Odometry()
    : pLeftEnc(nullptr), pRightEnc(nullptr),
    prevLeftCount(0), prevRightCount(0),
    lastLeftDelta(0), lastRightDelta(0),
    x(0), y(0), theta(0),
    segmentDist(0),
    linearVel(0), angularVel(0),
    leftTickRate(0), rightTickRate(0),
    lastUpdateUs(0)
{}

void Odometry::begin(ESP32Encoder* leftEnc, ESP32Encoder* rightEnc) {
    pLeftEnc  = leftEnc;
    pRightEnc = rightEnc;
    prevLeftCount  = leftEnc->getCount();
    prevRightCount = rightEnc->getCount();
    lastUpdateUs   = micros();
}

void Odometry::update() {
    if (!pLeftEnc || !pRightEnc) return;

    unsigned long nowUs = micros();
    float dtSec = (float)(nowUs - lastUpdateUs) / 1e6f;
    if (dtSec <= 0.0f) dtSec = 0.001f;   // guard against zero dt
    lastUpdateUs = nowUs;

    // Read encoder counts
    long leftCount  = pLeftEnc->getCount();
    long rightCount = pRightEnc->getCount();

    // Compute deltas
    lastLeftDelta  = leftCount  - prevLeftCount;
    lastRightDelta = rightCount - prevRightCount;
    prevLeftCount  = leftCount;
    prevRightCount = rightCount;

    // Per-wheel tick rates (used by velocity PID)
    leftTickRate  = (float)lastLeftDelta;
    rightTickRate = (float)lastRightDelta;

    // Convert to mm
    float dL = (float)lastLeftDelta  * MM_PER_TICK;
    float dR = (float)lastRightDelta * MM_PER_TICK;

    // Differential-drive kinematics
    float dDist  = (dL + dR) * 0.5f;
    float dTheta = (dR - dL) / WHEEL_BASE_MM;

    // Update pose (midpoint integration)
    float halfTheta = theta + dTheta * 0.5f;
    x     += dDist * cosf(halfTheta);
    y     += dDist * sinf(halfTheta);
    theta += dTheta;

    // Normalize theta to [-π, π]
    while (theta >  (float)M_PI)  theta -= 2.0f * (float)M_PI;
    while (theta < -(float)M_PI)  theta += 2.0f * (float)M_PI;

    // Segment distance (always positive, forward only)
    segmentDist += fabsf(dDist);

    // Velocities
    linearVel  = dDist  / dtSec;   // mm/s
    angularVel = dTheta / dtSec;   // rad/s
}

void Odometry::reset() {
    if (pLeftEnc)  prevLeftCount  = pLeftEnc->getCount();
    if (pRightEnc) prevRightCount = pRightEnc->getCount();
    x = y = theta = 0.0f;
    segmentDist = 0.0f;
    linearVel = angularVel = 0.0f;
    leftTickRate = rightTickRate = 0.0f;
    lastLeftDelta = lastRightDelta = 0;
    lastUpdateUs = micros();
}
