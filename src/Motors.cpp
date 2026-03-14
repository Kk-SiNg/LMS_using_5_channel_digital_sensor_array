/*
 * Motors.cpp
 * Motor control with per-wheel velocity PID (inner loop).
 * Uses QuickPID for robust anti-windup control.
 */

#include "Motors.h"
#include <Arduino.h>

// WiFi-tunable speed parameters (defined here, externed in Motors.h)
float cruiseSpeedMMS = DEFAULT_CRUISE_SPEED_MMS;
float maxSpeedMMS    = MAX_SPEED_MMS;
float turnSpeedMMS   = TURN_SPEED_MMS;

Motors::Motors()
    : leftTargetMMS(0), rightTargetMMS(0),
      leftTargetTicks(0), rightTargetTicks(0),
      leftMeasuredTicks(0), rightMeasuredTicks(0),
      leftPWMOutput(0), rightPWMOutput(0),
      prevLeftCount(0), prevRightCount(0),
      lastVelUpdateUs(0),
      // Initialize QuickPID with pointers to member variables
      leftVelPID(&leftMeasuredTicks, &leftPWMOutput, &leftTargetTicks,
                 VEL_KP_DEFAULT, VEL_KI_DEFAULT, VEL_KD_DEFAULT,
                 QuickPID::Action::direct),
      rightVelPID(&rightMeasuredTicks, &rightPWMOutput, &rightTargetTicks,
                  VEL_KP_DEFAULT, VEL_KI_DEFAULT, VEL_KD_DEFAULT,
                  QuickPID::Action::direct)
{}

void Motors::setup() {
    // Motor direction pins
    pinMode(MOTOR_L_AIN1, OUTPUT);
    pinMode(MOTOR_L_AIN2, OUTPUT);
    pinMode(MOTOR_R_BIN1, OUTPUT);
    pinMode(MOTOR_R_BIN2, OUTPUT);

    // PWM setup
    ledcSetup(pwm_channel_left, pwm_frequency, pwm_resolution);
    ledcSetup(pwm_channel_right, pwm_frequency, pwm_resolution);
    ledcAttachPin(MOTOR_L_PWMA, pwm_channel_left);
    ledcAttachPin(MOTOR_R_PWMB, pwm_channel_right);

    // Encoder setup
    puType pu_type = puType::none;
    ESP32Encoder::useInternalWeakPullResistors = pu_type;
    leftEncoder.attachHalfQuad(ENCODER_L_A, ENCODER_L_B);
    rightEncoder.attachHalfQuad(ENCODER_R_A, ENCODER_R_B);
    leftEncoder.clearCount();
    rightEncoder.clearCount();

    // Configure velocity PIDs
    leftVelPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
    rightVelPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
    leftVelPID.SetSampleTimeUs(CONTROL_INTERVAL_US);
    rightVelPID.SetSampleTimeUs(CONTROL_INTERVAL_US);
    leftVelPID.SetMode(QuickPID::Control::automatic);
    rightVelPID.SetMode(QuickPID::Control::automatic);

    prevLeftCount  = leftEncoder.getCount();
    prevRightCount = rightEncoder.getCount();
    lastVelUpdateUs = micros();

    stopBrake();
}

// =====================================================================
//  LOW-LEVEL PWM CONTROL
// =====================================================================

void Motors::applyPWM(int leftPWM, int rightPWM) {
    leftPWM  = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    // Left Motor direction
    if (leftPWM > 0) {
        digitalWrite(MOTOR_L_AIN1, HIGH);
        digitalWrite(MOTOR_L_AIN2, LOW);
    } else if (leftPWM < 0) {
        digitalWrite(MOTOR_L_AIN1, LOW);
        digitalWrite(MOTOR_L_AIN2, HIGH);
    } else {
        digitalWrite(MOTOR_L_AIN1, LOW);
        digitalWrite(MOTOR_L_AIN2, LOW);
    }
    ledcWrite(pwm_channel_left, abs(leftPWM));

    // Right Motor direction
    if (rightPWM > 0) {
        digitalWrite(MOTOR_R_BIN1, HIGH);
        digitalWrite(MOTOR_R_BIN2, LOW);
    } else if (rightPWM < 0) {
        digitalWrite(MOTOR_R_BIN1, LOW);
        digitalWrite(MOTOR_R_BIN2, HIGH);
    } else {
        digitalWrite(MOTOR_R_BIN1, LOW);
        digitalWrite(MOTOR_R_BIN2, LOW);
    }
    ledcWrite(pwm_channel_right, abs(rightPWM));
}

void Motors::setSpeeds(int leftPWM, int rightPWM) {
    applyPWM(leftPWM, rightPWM);
}

void Motors::stopBrake() {
    // Active braking: both direction pins HIGH, PWM 0
    digitalWrite(MOTOR_L_AIN1, HIGH);
    digitalWrite(MOTOR_L_AIN2, HIGH);
    digitalWrite(MOTOR_R_BIN1, HIGH);
    digitalWrite(MOTOR_R_BIN2, HIGH);
    ledcWrite(pwm_channel_left, 0);
    ledcWrite(pwm_channel_right, 0);

    // Reset PID targets
    leftTargetMMS = rightTargetMMS = 0;
    leftTargetTicks = rightTargetTicks = 0;
    leftPWMOutput = rightPWMOutput = 0;
}

// =====================================================================
//  VELOCITY PID (INNER LOOP)
// =====================================================================

float Motors::mmsToTicksPerInterval(float mms, float dtSec) {
    // Convert mm/s to ticks expected in one control interval
    return (mms * TICKS_PER_MM) * dtSec;
}

void Motors::setTargetVelocities(float leftMMS, float rightMMS) {
    leftTargetMMS  = constrain(leftMMS,  -maxSpeedMMS, maxSpeedMMS);
    rightTargetMMS = constrain(rightMMS, -maxSpeedMMS, maxSpeedMMS);
}

void Motors::updateVelocityPID() {
    unsigned long nowUs = micros();
    float dtSec = (float)(nowUs - lastVelUpdateUs) / 1e6f;
    if (dtSec <= 0.0f) dtSec = 0.001f;
    lastVelUpdateUs = nowUs;

    // Read encoder deltas
    long leftCount  = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();
    leftMeasuredTicks  = (float)(leftCount  - prevLeftCount);
    rightMeasuredTicks = (float)(rightCount - prevRightCount);
    prevLeftCount  = leftCount;
    prevRightCount = rightCount;

    // Convert target mm/s to ticks per this interval
    leftTargetTicks  = mmsToTicksPerInterval(leftTargetMMS, dtSec);
    rightTargetTicks = mmsToTicksPerInterval(rightTargetMMS, dtSec);

    // Compute PID
    leftVelPID.Compute();
    rightVelPID.Compute();

    // Apply
    applyPWM((int)leftPWMOutput, (int)rightPWMOutput);
}

void Motors::setVelPIDGains(float kp, float ki, float kd) {
    leftVelPID.SetTunings(kp, ki, kd);
    rightVelPID.SetTunings(kp, ki, kd);
}

// =====================================================================
//  UTILITY METHODS
// =====================================================================

void Motors::rotate() {
    // Open-loop spin for sensor calibration
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-130, 130);
}

void Motors::moveForward(int ticks) {
    // Blocking forward move for calibration/testing only
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(130, 130);
    while ((leftEncoder.getCount() + rightEncoder.getCount()) / 2 < ticks) {
        yield();
        delay(1);
    }
    stopBrake();
}

void Motors::clearEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    prevLeftCount  = 0;
    prevRightCount = 0;
}

// =====================================================================
//  WIFI TUNING
// =====================================================================

void Motors::updateSpeeds(float cruise, float turn, float maxSpd) {
    cruiseSpeedMMS = constrain(cruise, MIN_SPEED_MMS, MAX_SPEED_MMS);
    turnSpeedMMS   = constrain(turn,   MIN_SPEED_MMS, MAX_SPEED_MMS);
    maxSpeedMMS    = constrain(maxSpd,  MIN_SPEED_MMS, 1000.0f);
}