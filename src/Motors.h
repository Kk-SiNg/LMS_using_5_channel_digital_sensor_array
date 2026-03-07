/*
 * Motors.h
 * Enhanced motor control with WiFi-tunable parameters
 */

#pragma once
#include "Pins.h"
#include <ESP32Encoder.h>

class Sensors;

// WiFi-tunable motor parameters (defined as extern, initialized in Motors.cpp)
extern int TICKS_FOR_90_DEG;
extern int TICKS_FOR_180_DEG;
extern int TICKS_TO_CENTER;
extern int BASE_SPEED;
extern int TURN_SPEED;
extern int MAX_SPEED;
extern int MIN_TURN_PERCENT;  // Minimum % of ticks before sensor check kicks in
extern int BLIND_TURN_MS_90;
extern int BLIND_TURN_MS_180;
extern int TURN_TIMER_90;
extern int TURN_TIMER_180;


class Motors {
public:
    Motors();
    void setup();
    
    void setSpeeds(int leftSpeed, int rightSpeed);
    void stopBrake();
    
    // Original encoder-only turn methods
    void turn_90_left();
    void turn_90_right();
    void turn_180_back();

     // NEW: Smart turn methods with sensor feedback
    void turn_90_left_smart(Sensors& sensors);
    void turn_90_right_smart(Sensors& sensors);
    void turn_180_back_smart(Sensors& sensors);

    void rotate();
    void moveForward(int ticks);
    
    long getLeftCount();
    long getRightCount();
    long getAverageCount();
    void clearEncoders();
    
    // WiFi tuning methods
    static void updateTurn_90_Ticks(int ticks90);
    static void updateTurn_180_Ticks(int ticks180);

    static void updateCenterTicks(int ticks);
    static void updateSpeeds(int base, int turn, int max);

    static void updateMinTurnPercent(int percent);
    static void updateBlindTurnMs_180(int ms);
    static void updateBlindTurnMs_90(int ms);
    static void updateTurn_timer_90(int ms);
    static void update_turn_timer_180(int ms);

private:
    const int pwm_channel_left = 0;
    const int pwm_channel_right = 1;
    const int pwm_frequency = 5000;
    const int pwm_resolution = 8;
    
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;
};