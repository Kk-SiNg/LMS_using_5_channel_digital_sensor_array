/*
 * Sensors.h
 * 8-Channel QTRX Sensor Array
 * Using Pololu QTRSensors library with readLineWhite()
 */
#pragma once
#include "Pins.h"
#include <Arduino.h>
#include <QTRSensors.h>
#include <WiFi.h>

struct PathOptions {
    bool left = false;
    bool straight = false;
    bool right = false;
};

enum JunctionType {
    JUNCTION_NONE,
    JUNCTION_T_LEFT,
    JUNCTION_T_RIGHT,
    JUNCTION_T_BOTH,
    JUNCTION_CROSS,
    JUNCTION_90_LEFT,
    JUNCTION_90_RIGHT,
    JUNCTION_DEAD_END
};
class Sensors {
public:
    Sensors();
    void setup();
    
    float getLineError();
    PathOptions getAvailablePaths();
    PathOptions getAvailablePaths_2();
    JunctionType classifyJunction(PathOptions paths);
    bool isLineEnd();
    bool isEndPoint();
    
    void readRaw(uint16_t* values);
    void readDigital(bool* values);
    float getPosition();
    
    void getSensorArray(bool* arr);
    void getAnalogArray(uint16_t* arr);
    
    bool onLine();
    void printCalibration();
    void printCalibrationToClient(WiFiClient& client);
    
    int getActiveSensorCount();

    // ★★★ Sensitivity control ★★★
    void setSensitivity(float sens);   // 0.0 = max sensitive, 1.0 = least sensitive
    float getSensitivity();

private:
    QTRSensors qtr;
    uint16_t sensorValues[SensorCount];
    uint16_t calibratedThresholds[SensorCount];
    uint16_t activeThresholds[SensorCount];  // ★ actual thresholds used (adjusted by sensitivity)
    float sensitivity;                        // ★ 0.0 to 1.0
    
    const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    float lastPosition;
    
    bool isLineDetected(uint16_t value, uint8_t sensorIndex);
    void calculateThresholds();
    void recalculateActiveThresholds();  // ★ applies sensitivity to calibrated thresholds
};