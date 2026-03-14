/*
 * Sensors.h
 * 8-Channel QTRX Sensor Array
 * Simplified: provides line error, branch detection, and endpoint helpers.
 * Junction classification logic removed — handled by distance-confirmed
 * detection in main.cpp.
 */
#pragma once
#include "Pins.h"
#include "Config.h"
#include <Arduino.h>
#include <QTRSensors.h>
#include <WiFi.h>

class Sensors {
public:
    Sensors();
    void setup();

    // --- Line error (for steering PID) ---
    // Returns error in range -3500 to +3500 (center = 0)
    float getLineError();

    // --- Branch detection (instantaneous, must be distance-confirmed externally) ---
    // Returns true if the leftmost sensors (S7+S8) are active AND center sensors too
    bool hasLeftBranch();
    // Returns true if the rightmost sensors (S1+S2) are active AND center sensors too
    bool hasRightBranch();
    // Returns true if center sensors (S4+S5) are active (straight path available)
    bool hasStraight();

    // --- Line status ---
    bool isLineEnd();       // All sensors off → dead end
    bool isEndPoint();      // All sensors on  → finish marker
    bool onLine();          // Any sensor on

    // --- Raw data ---
    void readRaw(uint16_t* values);
    void readDigital(bool* values);
    float getPosition();
    void getSensorArray(bool* arr);
    void getAnalogArray(uint16_t* arr);
    int  getActiveSensorCount();
    uint8_t getActiveSensorMask();  // bit 0 = S1, bit 7 = S8

    // --- Sensitivity ---
    void setSensitivity(float sens);    // 0.0 = max sensitive, 1.0 = least
    float getSensitivity();

    // --- Calibration display ---
    void printCalibration();
    void printCalibrationToClient(WiFiClient& client);

private:
    QTRSensors qtr;
    uint16_t sensorValues[SensorCount];
    uint16_t calibratedThresholds[SensorCount];
    uint16_t activeThresholds[SensorCount];
    float sensitivity;

    float lastPosition;

    bool isLineDetected(uint16_t value, uint8_t sensorIndex);
    void calculateThresholds();
    void recalculateActiveThresholds();
};