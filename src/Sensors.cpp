/*
 * Sensors.cpp
 * 8-Channel QTRX Sensor Array
 * Using Pololu QTRSensors library with readLineWhite()
 * White lines on black background
 */

#include "Sensors.h"

// QTRX Configuration Constants
const uint16_t CALIBRATION_SAMPLES = 200;   // Number of calibration samples //400 to be set later
const uint16_t DEFAULT_THRESHOLD = 500;     // Default threshold before calibration
const uint16_t MIN_CALIBRATION_RANGE = 100; // Minimum difference between max and min for valid calibration

// Position constants for readLineWhite() which returns 0-7000
const float CENTER_POSITION = 3500.0;  // Center position on 8-sensor array

Sensors::Sensors() {
    lastPosition = 0.0;
    // Initialize thresholds to a default value
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibratedThresholds[i] = DEFAULT_THRESHOLD;  // Default fallback before calibration
    }
}

void Sensors::setup() {
    // Configure QTRX sensor array with 8 sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3, SENSOR_PIN_4, 
                                        SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7, SENSOR_PIN_8}, SensorCount);
    
    pinMode(ONBOARD_LED, OUTPUT);
    
    digitalWrite(ONBOARD_LED, HIGH);
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        qtr.calibrate();
    }
    digitalWrite(ONBOARD_LED, LOW);
    
    // Calculate thresholds from calibration data
    calculateThresholds();
}

void Sensors::printCalibration() {
    Serial.println("QTRX Calibration Values:");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": Min=");
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(" Max=");
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(" Threshold=");
        Serial.println(calibratedThresholds[i]);
    }
}

void Sensors::calculateThresholds() {
    // Calculate threshold for each sensor as the midpoint between min and max
    for (uint8_t i = 0; i < SensorCount; i++) {
        uint16_t minVal = qtr.calibrationOn.minimum[i];
        uint16_t maxVal = qtr.calibrationOn.maximum[i];
        
        // Validate calibration data - ensure min < max and sufficient range
        if (minVal >= maxVal || (maxVal - minVal) < MIN_CALIBRATION_RANGE) {
            // Invalid or insufficient calibration, use default threshold
            calibratedThresholds[i] = DEFAULT_THRESHOLD;
            Serial.print("Warning: Invalid/insufficient calibration for sensor ");
            Serial.print(i);
            Serial.print(" (min=");
            Serial.print(minVal);
            Serial.print(", max=");
            Serial.print(maxVal);
            Serial.println("), using default threshold");
            continue;
        }
        
        // Threshold = midpoint between minimum (black) and maximum (white)
        // Use uint32_t to prevent overflow when adding two uint16_t values
        calibratedThresholds[i] = (uint16_t)(((uint32_t)minVal + maxVal) / 2);
    }
    Serial.println("✓ Thresholds calculated from calibration data");
}

// Read all 8 sensors as analog values (0-1000)
// Higher values = more reflective (white line)
void Sensors::readRaw(uint16_t* values) {
    qtr.read(values);
}

void Sensors::readDigital(bool* values) {
    uint16_t rawValues[8];
    qtr.read(rawValues);
    
    // Convert analog readings to digital using calibrated thresholds
    // Values above threshold indicate white line
    for (uint8_t i = 0; i < 8; i++) {
        values[i] = (rawValues[i] > calibratedThresholds[i]);
    }
}

// ========== QTRX POSITION CALCULATION USING readLineWhite() ==========

float Sensors::getPosition() {
    // Use readLineWhite() which returns position from 0 to 7000
    // 0 = rightmost sensor, 7000 = leftmost sensor
    // CENTER_POSITION (3500) = center
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // Convert from 0-7000 scale to -7 to +7 scale
    // 0 -> -7 (right), 3500 -> 0 (center), 7000 -> +7 (left)
    lastPosition = ((float)position - CENTER_POSITION) / 500.0;
    
    return lastPosition;
}

float Sensors::getLineError() {
    // Use the raw position from readLineWhite() for more precise PID control
    // Returns error in range -3500 to +3500 (centered at 0)
    // 0 = rightmost, CENTER_POSITION = center (error = 0), 7000 = leftmost
    uint16_t position = qtr.readLineWhite(sensorValues);
    
    // Convert to error: center (CENTER_POSITION) = 0 error
    // Negative error = line is to the right, positive = line is to the left
    float error = (float)position - CENTER_POSITION;
    
    return error;
}

// ========== HELPER FUNCTIONS ==========

bool Sensors::isLineDetected(uint16_t value, uint8_t sensorIndex) {
    return (value > calibratedThresholds[sensorIndex]);  // Use calibrated threshold for each sensor
}

bool Sensors::onLine() {
    bool sensors[8];
    readDigital(sensors);
    
    // Check if ANY sensor sees the line
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            return true;
        }
    }
    return false;
}

int Sensors::getActiveSensorCount() {
    bool sensors[8];
    readDigital(sensors);
    
    int count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            count++;
        }
    }
    return count;
}

// ========== JUNCTION DETECTION ==========

PathOptions Sensors::getAvailablePaths() {
    bool sensors[8];
    readDigital(sensors);
    
    PathOptions paths;
    
    // Count active sensors (detecting white line)
    int activeCount = 0;
    for (int i = 0; i < 8; i++) {
        if (sensors[i]) activeCount++;
    }
    
    // Junction = 5 or more sensors active
    // Normal line = 2-3 sensors (as you described)
    if (activeCount >= 5) {
        // LEFT: S7 or S8 active
        paths.left = (sensors[6] && sensors[7]);
        
        // RIGHT: S1 or S2 active
        paths.right = (sensors[0] && sensors[1]);
        
        // STRAIGHT: S4 or S5 active (center)
        paths.straight = (sensors[3] && sensors[4]);
    }
    else {
        // Normal line - not a junction (2-3 sensors)
        paths.left = false;
        paths.right = false;
        paths.straight = (activeCount > 0);
    }
    
    return paths;
}

JunctionType Sensors::classifyJunction(PathOptions paths) {
    int pathCount = 0;
    if (paths.left) pathCount++;
    if (paths.straight) pathCount++;
    if (paths.right) pathCount++;
    
    if (pathCount == 0) {
        return JUNCTION_DEAD_END;
    } 
    else if (pathCount == 1) {
        if (paths.left && ! paths.straight) return JUNCTION_90_LEFT;
        if (paths.right && !paths.straight) return JUNCTION_90_RIGHT;
        return JUNCTION_NONE;
    } 
    else if (pathCount == 2) {
        if (paths.left && paths.straight) return JUNCTION_T_LEFT;
        if (paths.right && paths.straight) return JUNCTION_T_RIGHT;
        if (paths.left && paths.right) return JUNCTION_T_BOTH;
        return JUNCTION_NONE;
    } 
    else if (pathCount == 3) {
        return JUNCTION_CROSS;
    }
    
    return JUNCTION_NONE;
}

// ========== FINISH DETECTION ==========

bool Sensors::isLineEnd() {
    bool sensors[8];
    readDigital(sensors);
    
    // Line end = ALL sensors see black (none active)
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            return false;  // Still seeing white line
        }
    }
    return true;  // All sensors see black = line end
}

bool Sensors::isEndPoint() {
    bool sensors[8];
    readDigital(sensors);
    
    // Finish square
    for (uint8_t i = 0; i < 8; i++) {
        if (!sensors[i]) {
            return false;
        }
    }
    return true;
}

// ========== UTILITY ==========

void Sensors::getSensorArray(bool* arr) {
    readDigital(arr);
}

void Sensors::getAnalogArray(uint16_t* arr) {
    readRaw(arr);  // Returns actual analog values (0-1000)
}
