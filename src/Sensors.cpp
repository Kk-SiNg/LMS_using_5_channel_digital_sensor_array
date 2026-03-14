/*
 * Sensors.cpp
 * 8-Channel QTRX Sensor Array
 * Simplified for continuous PID-driven maze solving.
 */

#include "Sensors.h"

// QTRX calibration constants
const uint16_t CALIBRATION_SAMPLES = 200;
const uint16_t DEFAULT_THRESHOLD   = 500;
const float    CENTER_POSITION     = 3500.0f;

Sensors::Sensors() {
    lastPosition = 0.0f;
    sensitivity  = 0.45f;
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibratedThresholds[i] = DEFAULT_THRESHOLD;
    }
}

void Sensors::setup() {
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){
        SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3, SENSOR_PIN_4,
        SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7, SENSOR_PIN_8
    }, SensorCount);

    pinMode(ONBOARD_LED, OUTPUT);

    digitalWrite(ONBOARD_LED, HIGH);
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        qtr.calibrate();
    }
    digitalWrite(ONBOARD_LED, LOW);

    calculateThresholds();
    recalculateActiveThresholds();
}

// =====================================================================
//  LINE ERROR (for steering PID outer loop)
// =====================================================================

float Sensors::getLineError() {
    uint16_t position = qtr.readLineWhite(sensorValues);
    float error = (float)position - CENTER_POSITION;
    return error;   // range: -3500 to +3500
}

float Sensors::getPosition() {
    uint16_t position = qtr.readLineWhite(sensorValues);
    lastPosition = ((float)position - CENTER_POSITION) / 500.0f;
    return lastPosition;
}

// =====================================================================
//  BRANCH DETECTION (instantaneous)
//  These are raw sensor checks.  Distance-confirmation happens in main.cpp.
// =====================================================================

bool Sensors::hasLeftBranch() {
    bool s[8];
    readDigital(s);
    // Leftmost sensors S7+S8 active AND at least one center sensor
    return (s[6] && s[7]) && (s[3] || s[4]);
}

bool Sensors::hasRightBranch() {
    bool s[8];
    readDigital(s);
    // Rightmost sensors S1+S2 active AND at least one center sensor
    return (s[0] && s[1]) && (s[3] || s[4]);
}

bool Sensors::hasStraight() {
    bool s[8];
    readDigital(s);
    return s[3] && s[4];
}

// =====================================================================
//  LINE STATUS
// =====================================================================

bool Sensors::isLineEnd() {
    bool s[8];
    readDigital(s);
    for (uint8_t i = 0; i < 8; i++) {
        if (s[i]) return false;  // at least one sensor sees line
    }
    return true;   // all sensors off → dead end
}

bool Sensors::isEndPoint() {
    bool s[8];
    readDigital(s);
    for (uint8_t i = 0; i < 8; i++) {
        if (!s[i]) return false;
    }
    return true;   // all sensors on → finish marker
}

bool Sensors::onLine() {
    bool s[8];
    readDigital(s);
    for (uint8_t i = 0; i < 8; i++) {
        if (s[i]) return true;
    }
    return false;
}

// =====================================================================
//  RAW DATA
// =====================================================================

void Sensors::readRaw(uint16_t* values) {
    qtr.read(values);
}

void Sensors::readDigital(bool* values) {
    uint16_t rawValues[8];
    qtr.read(rawValues);
    for (uint8_t i = 0; i < 8; i++) {
        values[i] = (rawValues[i] < activeThresholds[i]);
    }
}

void Sensors::getSensorArray(bool* arr) {
    readDigital(arr);
}

void Sensors::getAnalogArray(uint16_t* arr) {
    readRaw(arr);
}

int Sensors::getActiveSensorCount() {
    bool s[8];
    readDigital(s);
    int count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (s[i]) count++;
    }
    return count;
}

uint8_t Sensors::getActiveSensorMask() {
    bool s[8];
    readDigital(s);
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (s[i]) mask |= (1 << i);
    }
    return mask;
}

// =====================================================================
//  CALIBRATION & SENSITIVITY
// =====================================================================

void Sensors::calculateThresholds() {
    for (uint8_t i = 0; i < SensorCount; i++) {
        uint16_t minVal = qtr.calibrationOn.minimum[i];
        uint16_t maxVal = qtr.calibrationOn.maximum[i];
        calibratedThresholds[i] = (uint16_t)(((uint32_t)minVal + maxVal) / 2);
    }
    Serial.println("✓ Thresholds calculated from calibration data");
}

void Sensors::recalculateActiveThresholds() {
    for (uint8_t i = 0; i < SensorCount; i++) {
        uint16_t minVal = qtr.calibrationOn.minimum[i];
        uint16_t midVal = calibratedThresholds[i];
        float range = (float)(midVal - minVal);
        activeThresholds[i] = (uint16_t)(midVal - (sensitivity * range));
        if (activeThresholds[i] < minVal) {
            activeThresholds[i] = minVal;
        }
    }
}

void Sensors::setSensitivity(float sens) {
    sensitivity = constrain(sens, 0.0f, 1.0f);
    recalculateActiveThresholds();
}

float Sensors::getSensitivity() {
    return sensitivity;
}

bool Sensors::isLineDetected(uint16_t value, uint8_t sensorIndex) {
    return (value < activeThresholds[sensorIndex]);
}

// =====================================================================
//  CALIBRATION DISPLAY
// =====================================================================

void Sensors::printCalibration() {
    Serial.println("QTRX Calibration Values:");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.printf("Sensor %d: Min=%d Max=%d CalThresh=%d ActiveThresh=%d\n",
            i, qtr.calibrationOn.minimum[i], qtr.calibrationOn.maximum[i],
            calibratedThresholds[i], activeThresholds[i]);
    }
    Serial.printf("Sensitivity: %.2f\n", sensitivity);
}

void Sensors::printCalibrationToClient(WiFiClient& client) {
    client.println("\n=== Calibration Thresholds ===");
    client.printf("Sensitivity: %.2f (0=max sensitive, 1=least)\n", sensitivity);
    client.println("Sensor | Min    | Max    | CalThr | ActiveThr");
    client.println("-------|--------|--------|--------|----------");
    for (uint8_t i = 0; i < SensorCount; i++) {
        client.printf("  S%-2d  | %-6d | %-6d | %-6d | %-6d\n",
            i + 1,
            qtr.calibrationOn.minimum[i],
            qtr.calibrationOn.maximum[i],
            calibratedThresholds[i],
            activeThresholds[i]);
    }
    client.println("==============================\n");
}
