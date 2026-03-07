/*
 * Motors.cpp
 * Motor control with WiFi-tunable parameters
 * V3.2 - Added smart turn methods with sensor feedback
 */

#include "Motors.h"
#include "Sensors.h"
#include <Arduino.h>

// Define global tunable parameters can be changed externally via wifi
int TICKS_FOR_90_DEG = 450;    //old 360
int TICKS_FOR_180_DEG = 800;
int TICKS_TO_CENTER = 150;       //old 100
int BASE_SPEED = 130;
int TURN_SPEED = 125;
int MAX_SPEED = 200;
int MIN_TURN_PERCENT = 80;  // Start checking sensors after 65% of turn complete

Motors::Motors() {}

void Motors::setup() {
    pinMode(MOTOR_L_AIN1, OUTPUT);
    pinMode(MOTOR_L_AIN2, OUTPUT);
    pinMode(MOTOR_R_BIN1, OUTPUT);
    pinMode(MOTOR_R_BIN2, OUTPUT);
    
    ledcSetup(pwm_channel_left, pwm_frequency, pwm_resolution);
    ledcSetup(pwm_channel_right, pwm_frequency, pwm_resolution);
    
    ledcAttachPin(MOTOR_L_PWMA, pwm_channel_left);
    ledcAttachPin(MOTOR_R_PWMB, pwm_channel_right);
    
    puType pu_type = puType::none;
    ESP32Encoder::useInternalWeakPullResistors = pu_type;
    leftEncoder.attachHalfQuad(ENCODER_L_A, ENCODER_L_B);
    rightEncoder.attachHalfQuad(ENCODER_R_A, ENCODER_R_B);
    
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    stopBrake();
}

void Motors::setSpeeds(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Left Motor
    if (leftSpeed > 0) {
        digitalWrite(MOTOR_L_AIN1, HIGH);
        digitalWrite(MOTOR_L_AIN2, LOW);
    } 
    else if (leftSpeed < 0) {
        digitalWrite(MOTOR_L_AIN1, LOW);
        digitalWrite(MOTOR_L_AIN2, HIGH);
    } 
    else {
        digitalWrite(MOTOR_L_AIN1, LOW);
        digitalWrite(MOTOR_L_AIN2, LOW);
    }
    ledcWrite(pwm_channel_left, abs(leftSpeed));
    
    // Right Motor
    if (rightSpeed > 0) {
        digitalWrite(MOTOR_R_BIN1, HIGH);
        digitalWrite(MOTOR_R_BIN2, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(MOTOR_R_BIN1, LOW);
        digitalWrite(MOTOR_R_BIN2, HIGH);
    } else {
        digitalWrite(MOTOR_R_BIN1, LOW);
        digitalWrite(MOTOR_R_BIN2, LOW);
    }
    ledcWrite(pwm_channel_right, abs(rightSpeed));
}

void Motors::stopBrake() {
    digitalWrite(MOTOR_L_AIN1, HIGH);
    digitalWrite(MOTOR_L_AIN2, HIGH);
    digitalWrite(MOTOR_R_BIN1, HIGH);
    digitalWrite(MOTOR_R_BIN2, HIGH);
    ledcWrite(pwm_channel_left, 0);
    ledcWrite(pwm_channel_right, 0);
}

// ========== ORIGINAL ENCODER-ONLY TURN METHODS ==========

void Motors::turn_90_left() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-TURN_SPEED, TURN_SPEED);
    while (rightEncoder.getCount() < TICKS_FOR_90_DEG) delay(1);
    stopBrake();
}

void Motors::turn_90_right() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(TURN_SPEED, -TURN_SPEED);
    while (leftEncoder.getCount() < TICKS_FOR_90_DEG) delay(1);
    stopBrake();
}

void Motors::turn_180_back() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(TURN_SPEED, -TURN_SPEED);     //pivot from right
    while (leftEncoder.getCount() < TICKS_FOR_180_DEG) delay(1);
    stopBrake();
}

// ========== NEW:  SMART TURN METHODS WITH SENSOR FEEDBACK ==========

void Motors::turn_90_left_smart(Sensors& sensors) {
    setSpeeds(-TURN_SPEED, TURN_SPEED);
    
    // Calculate minimum ticks before we start checking sensors
    long minTicks = (TICKS_FOR_90_DEG * MIN_TURN_PERCENT) / 100;
    
    while (rightEncoder.getCount() < TICKS_FOR_90_DEG) {
        // After minimum ticks, check if center sensors see the line
        if (rightEncoder.getCount() >= minTicks) {
            bool sensorVals[8];
            sensors.getSensorArray(sensorVals);
            
            // Check center sensors (S4 and S5 - indices 3 and 4)
            // These are the primary line-following sensors
            if ((sensorVals[3] && sensorVals[4]) || (sensorVals[4] && sensorVals[5]) || (sensorVals[5] && sensorVals[6]) || (sensorVals[6] && sensorVals[7])) {
                break;  // Exit early - line found!
            }
        }
        yield();
        delay(1);
    }
    stopBrake();
}

void Motors::turn_90_right_smart(Sensors& sensors) {

    setSpeeds(TURN_SPEED, -TURN_SPEED);
    
    // Calculate minimum ticks before we start checking sensors
    long minTicks = (TICKS_FOR_90_DEG * MIN_TURN_PERCENT) / 100;
    
    while (leftEncoder.getCount() < TICKS_FOR_90_DEG) {
        // After minimum ticks, check if center sensors see the line
        if (leftEncoder.getCount() >= minTicks) {
            bool sensorVals[8];
            sensors.getSensorArray(sensorVals);
            
            // Check center sensors (S4 and S5 - indices 3 and 4)
            if ((sensorVals[0] && sensorVals[1]) || (sensorVals[1] && sensorVals[2]) || (sensorVals[2] && sensorVals[3]) || (sensorVals[3] && sensorVals[4])) {
                break;  // Exit early - line found!
            }
        }
        yield();
        delay(1);
    }
    stopBrake();
}

void Motors::turn_180_back_smart(Sensors& sensors) {

    setSpeeds(TURN_SPEED, -TURN_SPEED);  // pivot from right
    
    // Calculate minimum ticks before we start checking sensors
    long minTicks = (TICKS_FOR_180_DEG * MIN_TURN_PERCENT) / 100;
    
    while (leftEncoder.getCount() < TICKS_FOR_180_DEG) {
        // After minimum ticks, check if center sensors see the line
        if (leftEncoder.getCount() >= minTicks) {
            bool sensorVals[8];
            sensors.getSensorArray(sensorVals);
            
            // Check center sensors (S4 and S5 - indices 3 and 4)
            // For 180° turns, we want to catch the line coming from behind
            if ((sensorVals[1] && sensorVals[2]) || (sensorVals[2] && sensorVals[3]) || (sensorVals[3] && sensorVals[4])) {
                break;  // Exit early - line found! 
            }
        }
        yield();
        delay(1);
    }
    stopBrake();
}

// ========== UTILITY METHODS ==========

void Motors::rotate() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-130, 130);
}

void Motors::moveForward(int ticks) {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(BASE_SPEED, BASE_SPEED);
    while ((leftEncoder.getCount() + rightEncoder.getCount()) / 2 < ticks) {
        yield();
        delay(1);
    }

    stopBrake();
}

long Motors::getLeftCount() {
    return leftEncoder.getCount();
}

long Motors::getRightCount() {
    return rightEncoder.getCount();
}

long Motors::getAverageCount() {
    return (long)(leftEncoder.getCount() + rightEncoder.getCount()) / 2;
}

void Motors::clearEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

// ========== WIFI TUNING METHODS ==========

void Motors::updateTurn_90_Ticks(int ticks90) {
    TICKS_FOR_90_DEG = constrain(ticks90, 25, 1000);
}

void Motors::updateTurn_180_Ticks(int ticks180) {
    TICKS_FOR_180_DEG = constrain(ticks180, 100, 2000);
}

void Motors::updateCenterTicks(int ticks) {
    TICKS_TO_CENTER = constrain(ticks, 10, 500);
}

void Motors::updateSpeeds(int base, int turn, int max) {
    BASE_SPEED = constrain(base, 50, 255);
    TURN_SPEED = constrain(turn, 50, 255);
    MAX_SPEED = constrain(max, 100, 255);
}

void Motors::updateMinTurnPercent(int percent) {
    MIN_TURN_PERCENT = constrain(percent, 30, 95);
}