#include <Arduino.h>
/*
 * IIT Bombay Mesmerize Line Follower
 * Fresh implementation with digital sensors
 * Simple PID + LSRB junction navigation
 * WITH PATH SAVING & OPTIMIZATION FEATURES
 * V3.1 - Corrected Dynamic Debouncing
 */

#include <WiFi.h>
#include "Pins.h"
#include "Sensors.h"
#include "Motors.h"
#include "PathOptimization.h"

// Access motor tick parameters for WiFi tuning
extern int TICKS_TO_CENTER;
extern int TICKS_FOR_90_DEG;
extern int MIN_TURN_PERCENT;


// === WiFi Server ===
WiFiServer server(TELNET_PORT);
WiFiClient client;

// === Global Objects ===
Sensors sensors;
Motors motors;
PathOptimization optimizer;

// === Simple PID Variables ===
// Updated for 0-7000 position range (error range: -3500 to +3500)
// Previous values for -7 to +7 range: Kp=15.0, Ki=0.0, Kd=0.7
// New values scaled down by 500x to maintain similar behavior
float Kp = 0.03;    // Proportional gain (15.0 / 500)
float Ki = 0.0;     // Integral gain (start with 0)
float Kd = 0.0014;  // Derivative gain (0.7 / 500)
float lastError = 0;
float integral = 0;
float maxIntegral = 500000;  // Scaled for new error range (1000 * 500). Adjust if enabling Ki.

int baseSpeed = 70;    // general base speed for normal runs
int maxSpeed = 90;     //max speed during run
int highSpeed = 140;  // For solving case

//Addition
int junction_identification_delay = 0; //move these many ticks to reverify junction and get available paths
int line_end_confirmation_ticks = 5;

// Delays
int delayBeforeCenter = 1000;
int delayAfterCenter = 1000;
int dl1 = 1000;
int dl2 = 1000;
int dl3 = 1000;
int dl4 = 1000;
int dl5 = 1000;

// // Alignment correction
// int adjusted_speed = 100;
// int align_corr_time = 100;
// float correction_multiplier = 1.5;
// int initial_ticks = 0;

// //new extra movement for alignment correction
// int extra_ticks = 0;

// === Junction Settings ===
unsigned long junctionDebounce = 120;  // ms between junction detections
unsigned long lastJunctionTime = 0;
int junctionCount = 0;

// === PATH SAVING CONSTANTS ===
#define MAX_PATH_LENGTH 100

int SLOWDOWN_TICKS = 85;  // Ticks before junction to slow down in optimized run

// === PATH STORAGE ===
String rawPath = "";
long pathSegments[MAX_PATH_LENGTH];
int pathIndex = 0;

String optimizedPath = "";
long optimizedSegments[MAX_PATH_LENGTH];
int optimizedPathLength = 0;
int solvePathIndex = 0;

// === Finish Detection ===
unsigned long finishDetectTime = 0;
const unsigned long FINISH_CONFIRM_MS = 300;  // Confirm finish for 300ms

// === Robot State Machine ===
enum State {
    CALIBRATING,
    WAIT_FOR_RUN_1,       // Waiting to start mapping run
    MAPPING,              // First run - mapping the maze
    OPTIMIZING,           // Processing path
    WAIT_FOR_RUN_2,       // Waiting to start optimized run
    SOLVING,              // Second run - using optimized path
    FINISHED
};
State currentState = CALIBRATING;

// === Solving Sub-State Machine ===
enum SolvingSubState {
    SOLVE_TURN,
    SOLVE_FAST_RUN,
    SOLVE_SLOW_RUN,
    SOLVE_FINAL_RUN
};
SolvingSubState solveState = SOLVE_TURN;

bool robotRunning = false;

// === Timing ===
unsigned long runStartTime = 0;
unsigned long mappingStartTime = 0;
unsigned long solvingStartTime = 0;
unsigned long lastWiFiUpdate = 0;
unsigned long lastDebugPrint = 0;


// === Emergency Stop ===
unsigned long buttonPressStart = 0;

// === Line End Detection ===
unsigned long lineEndStartTime = 0;
const unsigned long LINE_END_CONFIRM_TIME = 150;

// === Performance Metrics ===
// float avgSegmentLength = 0.0;
// long totalSegmentTicks = 0;

// === Function Declarations ===
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();
void printMotorParams();
void runPID(int speed, float correction_multiplier = 1.0f);
String junctionTypeToString(JunctionType type);
unsigned long getDynamicDebounce();

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  IIT Bombay Mesmerize Line Follower  ║");
    Serial.println("║  Digital Sensors + Path Optimization  ║");
    Serial.println("║  V3.1 - Corrected Debouncing          ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    pinMode(ONBOARD_LED, OUTPUT);
    // pinMode(RGB_PIN_R, OUTPUT);
    // pinMode(RGB_PIN_G, OUTPUT);
    // pinMode(RGB_PIN_B, OUTPUT);
    
    pinMode(USER_BUTTON, INPUT_PULLUP);
    
    // Initialize motors and sensors
    motors.setup();
    
    // Calibrate while rotating (gives sensor exposure to both surfaces)
    Serial.println("Starting sensor calibration...");
    setupWiFi();
    while (true){

        handleWiFiClient();
        yield();
        
        if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
            delay(50);  // Debounce
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) break;
        }
    }
    currentState = CALIBRATING;
    motors.rotate();
    sensors.setup();
    motors.stopBrake();
    // WiFi.mode(WIFI_OFF);  // Turn off Wi-Fi completely
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  Configuration                         ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║  PID: Kp=%.1f Ki=%.1f Kd=%.1f          ║\n", Kp, Ki, Kd);
    Serial.printf("║  Base Speed: %d  High Speed: %d      ║\n", BASE_SPEED, highSpeed);
    Serial.printf("║  Junction Debounce: %lums              ║\n", junctionDebounce);
    Serial.printf("║  TICKS_90° = %d  Center = %d         ║\n", TICKS_FOR_90_DEG, TICKS_TO_CENTER);
    Serial.println("╚════════════════════════════════════════╝\n");
    
    currentState = WAIT_FOR_RUN_1;
    Serial.println("✓ Ready for Run 1 (Mapping)");
    Serial.println("Press button or type START via WiFi\n");
}

unsigned long getDynamicDebounce() {
    // === CORRECTED LOGIC ===
    // Slower speed = LONGER debounce (robot stays on junction longer)
    // Faster speed = SHORTER debounce (robot crosses junction quickly)
    
    // Prevent division by zero
    if (baseSpeed == 0) return junctionDebounce;
    
    // Inverted ratio: BASE_SPEED / currentSpeed
    float speedRatio = (float)(200-50) / (float)baseSpeed;
    unsigned long dynamicValue = (unsigned long)(junctionDebounce * speedRatio);
    
    // Clamp to reasonable range
    return constrain(dynamicValue, 150, 800);
}

void loop() {
    yield();  // Let WiFi handle its tasks
    handleWiFiClient();
    
    // === Enhanced WiFi Telemetry (every 500ms) ===
    if (millis() - lastWiFiUpdate > 1000 && client && client.connected()) {
        if (client.availableForWrite() > 100) {
            bool sensorVals[8];
            sensors.getSensorArray(sensorVals);
            
            client.print("S:[");
            for (int i = 0; i < 8; i++) {
                client.print(sensorVals[i] ? "█" : "·");
            }
            
            unsigned long dynDebounce = getDynamicDebounce();
            client.printf("] Err:%.1f Spd:%d DB:%lums | ", 
                        sensors.getLineError(), baseSpeed, dynDebounce);
            
            switch(currentState) {
                case WAIT_FOR_RUN_1: client.print("WAIT_RUN1"); break;
                case MAPPING: client.print("MAPPING"); break;
                case OPTIMIZING: client.print("OPTIMIZING"); break;
                case WAIT_FOR_RUN_2: client.print("WAIT_RUN2"); break;
                case SOLVING: client.print("SOLVING"); break;
                case FINISHED: client.print("FINISHED"); break;
                default: client.print("CALIBRATING");
            }
            
            if(currentState == MAPPING || currentState == SOLVING) {
                client.print(" | Junc:");
                client.print(pathIndex);
                if(currentState == SOLVING && optimizedPathLength > 0) {
                    client.print("/");
                    client.print(optimizedPathLength);
                }
            }
            
            client.println();
            client.flush();
        }
        lastWiFiUpdate = millis();
    }
    
    // === Emergency Stop (2 second button press) ===
    if (digitalRead(USER_BUTTON) == LOW) {
        if (buttonPressStart == 0) {
            buttonPressStart = millis();
        } else if (millis() - buttonPressStart > 2000) {
            motors.stopBrake();
            robotRunning = false;
            currentState = FINISHED;
            Serial.println("\n⚠️ EMERGENCY STOP!");
            if (client && client.connected()) {
                client.println("⚠️ EMERGENCY STOP!");
            }
            buttonPressStart = 0;
        }
    }
    else {
        buttonPressStart = 0;
    }

    // === Detailed Debug Output (every 100ms) ===
    if (millis() - lastDebugPrint > 1000) {
        bool sensorVals[8];
        sensors.getSensorArray(sensorVals);

        if (client && client.connected()){
            
            client.print("Sensor: [");
            for(int i = 0; i < 8; i++) {
                client.print(sensorVals[i] ? "█" : "·");
            }
            client.printf("] Err:%.1f Speed:%d Path:%s DynDB:%lums\n", 
                            sensors.getLineError(), baseSpeed, rawPath.c_str(), getDynamicDebounce());
            client.println();
            
        }
        lastDebugPrint = millis();
    }
    
    // === Main State Machine ===
    switch (currentState) {
        
        case WAIT_FOR_RUN_1:
        {
            // digitalWrite(RGB_PIN_R, LOW);
            // digitalWrite(RGB_PIN_B, HIGH);
            // digitalWrite(RGB_PIN_G, LOW);

            // Wait for button press or START command
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                delay(50);  // Debounce
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    
                    if (client && client.connected()) {
                        client.println("\n>>> RUN 1: MAPPING STARTED!");
                    }
                    
                    currentState = MAPPING;
                    robotRunning = true;
                    pathIndex = 0;
                    rawPath = "S";
                    // totalSegmentTicks = 0;
                    junctionCount = 0;
                    lastJunctionTime = 0;
                    lineEndStartTime = 0;
                    finishDetectTime = 0;
                    mappingStartTime = millis();
                    
                    // Reset PID
                    lastError = 0;
                    integral = 0;
                    motors.clearEncoders();
                    
                    // Wait for button release
                    while(digitalRead(USER_BUTTON) == LOW) delay(10);

                    // digitalWrite(RGB_PIN_R, LOW);
                    // digitalWrite(RGB_PIN_B, LOW);
                    // digitalWrite(RGB_PIN_G, HIGH);
                }
            }
            break;
        }
            
        case MAPPING:
        {
            if (!robotRunning) {
                motors.stopBrake();
                currentState = FINISHED;
                if (client && client.connected()) client.println("\n>>> STOPPED");
                break;
            }
            
            // === Run PID Line Following ===
            runPID(baseSpeed);

            // === Junction Detection (with dynamic debounce) ===
            unsigned long currentDebounce = getDynamicDebounce();
            
            if (millis() - lastJunctionTime > currentDebounce) {
                PathOptions pathsA1 = sensors.getAvailablePaths();
                
                // Count available pathsA
                int pathCount = 0;
                if (pathsA1.left) pathCount++;
                if (pathsA1.right) pathCount++;
                if (pathsA1.straight) pathCount++;
                
                // Is this a junction?  (more than just straight OR only left/right)
                bool isJunction_check_1 = (pathCount > 1) || (pathCount == 1 && !pathsA1.straight);
                bool isjunction_check_2 = 0;    
                if(isJunction_check_1){
                    int confirmations = 0;
                    for (int i = 0; i < 5; i++) {  // Take 5 quick samples
                        PathOptions p = sensors.getAvailablePaths();
                        int pc = 0;
                        if (p.left) pc++;
                        if (p.right) pc++;
                        if (p.straight) pc++;
                        if ((pc > 1) || (pc == 1 && !p.straight)) {
                            confirmations++;
                        }
                        delayMicroseconds(500);  // 0.5ms between samples
                    }
                    if(confirmations > 3) isjunction_check_2 = true;
                    else isjunction_check_2 = false;
                }
                    

                // if (client && client.connected()) client.printf("leftA: %d, rightA: %d \n", pathsA.left, pathsA.right);
                // if (client && client.connected()) client.println();
                if (isJunction_check_1 && isjunction_check_2) {
                    // Record segment ticks BEFORE any junction handling
                    long segmentTicks = motors.getAverageCount();
                    
                    motors.stopBrake();
                    delay(1);
                    
                    // Continuous path detection for 100ms WITHOUT PID (to avoid drift)
                    unsigned long detectionStartTime = millis();
                    int leftDetections = 0;
                    int rightDetections = 0;
                    int straightDetections = 0;
                    int totalSamples = 0;
                    
                    // Track starting position for the 100ms sampling movement
                    long samplingStartTicks = motors.getAverageCount();
                    
                    while (millis() - detectionStartTime < 100) {  // 100ms continuous detection
                        // Just move straight slowly, NO PID correction
                        motors.setSpeeds(80, 80);  // Equal speeds = straight movement
                        
                        PathOptions sample = sensors.getAvailablePaths_2();
                        if (sample.left) leftDetections++;
                        if (sample.right) rightDetections++;
                        if (sample.straight) straightDetections++;
                        totalSamples++;
                        
                        yield();
                        delay(3);  // Sample every 3ms
                    }
                    
                    motors.stopBrake();
                    
                    // Calculate how many ticks were traveled during sampling
                    long samplingTicks = motors.getAverageCount() - samplingStartTicks;
                    
                    if (client && client.connected()) {
                        client.println("Junction detected!\n");
                        client.printf("Samples: L=%d, S=%d, R=%d (Total=%d)\n", 
                                    leftDetections, straightDetections, rightDetections, totalSamples);
                        client.printf("Ticks during sampling:  %ld\n", samplingTicks);
                    }
                    
                    // Combine results with confidence threshold
                    PathOptions paths;
                    float leftConfidence = (totalSamples > 0) ? (float)leftDetections / totalSamples : 0;
                    float rightConfidence = (totalSamples > 0) ? (float)rightDetections / totalSamples : 0;
                    float straightConfidence = (totalSamples > 0) ? (float)straightDetections / totalSamples : 0;
                    
                    paths.left = (leftConfidence >= 0.1);  // 10% confidence threshold
                    paths.right = (rightConfidence >= 0.1);
                    paths.straight = (straightConfidence >= 0.85);  // 85% confidence threshold
                    
                    if (client && client.connected()) {
                        client.printf("Path confidence - L:  %.0f%%, S: %. 0f%%, R: %.0f%%\n",
                                    leftConfidence * 100, straightConfidence * 100, rightConfidence * 100);
                        client.printf("Detected paths - L: %d, S: %d, R: %d\n",
                                    paths.left, paths.straight, paths.right);
                    }
                    
                    // // Move forward additional amount if needed (junction_identification_delay)
                    // if (junction_identification_delay > 0) {
                    //     if (client && client.connected()) {
                    //         client.printf("Moving forward by %d ticks\n", junction_identification_delay);
                    //     }
                    //     motors.moveForward(junction_identification_delay);
                    //     motors.stopBrake();
                    // }
                    
                    if (client && client.connected()) client.println("Taking delay before centering");
                    delay(delayBeforeCenter);
                    
                    // Display junction info
                    if (client && client. connected()) {
                        client.printf("J%d: ", junctionCount);
                        if (paths.left) client.print("L");
                        if (paths.straight) client.print("S");
                        if (paths.right) client.print("R");
                        client.println();
                    }
                    
                    // Move to center
                    if (client && client.connected()) client.println("Executing ticks to center");
                    motors.moveForward(TICKS_TO_CENTER);
                    motors.stopBrake();
                    yield();
                    
                    if (client && client.connected()) client.println("Taking Delaying after center");
                    delay(delayAfterCenter);
                    yield();

                    // Calculate total segment length
                    long totalSegmentLength = segmentTicks + samplingTicks + junction_identification_delay + TICKS_TO_CENTER;

                    if (client && client. connected()) {
                        client.printf("Segment length:  %ld ticks\n", totalSegmentLength);
                    }
                    
                    // Check for overflow
                    if (pathIndex >= MAX_PATH_LENGTH) {
                        Serial.println("❌ ERROR: Path array full!");

                        // digitalWrite(RGB_PIN_R, HIGH);
                        // digitalWrite(RGB_PIN_B, LOW);
                        // digitalWrite(RGB_PIN_G, LOW);

                        currentState = FINISHED;
                        break;
                    }
                    
                    // Save segment
                    pathSegments[pathIndex] = totalSegmentLength;
                    
                    // Clear encoders BEFORE turn
                    motors.clearEncoders();
                    delay(1);
                    
                    junctionCount++;
                    
                    // Check for endpoint
                    if (sensors.isEndPoint()) {
                        motors.stopBrake();
                        robotRunning = false;
                        
                        unsigned long runTime = (millis() - mappingStartTime) / 1000;
                        
                        if (client && client.connected()) {
                            client.println("\n🏆 DRY RUN COMPLETE!");
                            client.printf("Time: %lus | Junctions: %d\n", runTime, junctionCount);
                            client.println();
                        }

                        motors.moveForward(150);
                        motors.stopBrake();

                        // digitalWrite(RGB_PIN_R, HIGH);
                        // digitalWrite(RGB_PIN_B, LOW);
                        // digitalWrite(RGB_PIN_G, LOW);

                        currentState = OPTIMIZING;
                        // digitalWrite(RGB_PIN_R, HIGH);
                        break;
                    }
                    
                    bool sensorVals[8];
                    sensors.getSensorArray(sensorVals);
                    
                    // === LSRB Logic:  Left > Straight > Right > Back ===
                    if (paths.left) {
                        if (client && client.connected()) client.println("  → Taking LEFT");
                        motors.turn_90_left_smart(sensors);
                        rawPath += 'L';
                    }
                    else if ((sensorVals[3] || sensorVals[4]) || paths.straight) {
                        if (client && client.connected()) client.println("  → Going STRAIGHT");
                        rawPath += 'S';
                    }
                    else if (paths.right) {
                        if (client && client.connected()) client.println("  → Taking RIGHT");
                        motors.turn_90_right_smart(sensors);
                        rawPath += 'R';
                    }
                    else {
                        if (client && client.connected()) client.println("  → DEAD END - Turning back");
                        motors.turn_180_back_smart(sensors);
                        rawPath += 'B';
                    }

                    // motors.clearEncoders();
                    // // Alignment Correction
                    // int time_taken = millis();
                    // while ((millis() - time_taken) < align_corr_time) {
                    //     runPID(adjusted_speed, correction_multiplier);
                    // }
                    // extra_ticks = motors.getAverageCount();
                    // motors.stopBrake();
                    // delay(1000);

                    
                    pathIndex++;
                    
                    // Reset after junction
                    motors.clearEncoders();
                    lastError = sensors.getLineError();  // Use current error, not 0
                    integral = 0;
                    lastJunctionTime = millis();
                    delay(dl1);
                    yield();
                }
                else if (sensors.isLineEnd()) {
                        
                    long segmentTicks = motors.getAverageCount();
                    motors.moveForward(line_end_confirmation_ticks);
                    motors.stopBrake();
                    delay(dl2);

                    if (sensors.isLineEnd()) {
                        if (client && client.connected()) client.println("  → DEAD END - Turning back");
                        junctionCount++;

                        // MOVE TO CENTER
                        motors.moveForward(20);

                        // Record and Save segment for backing up
                        pathSegments[pathIndex] = segmentTicks + 20 + line_end_confirmation_ticks;

                        pathIndex++;

                        // Turn and save path
                        motors.turn_180_back_smart(sensors);
                        rawPath += 'B';

                        // Reset
                        motors.clearEncoders();
                        lastError = 0;
                        integral = 0;
                        lastJunctionTime = millis();
                        delay(dl1);
                    }
                    else {
                        if (client && client. connected()) client.println("false line end detected");
                        runPID(baseSpeed);
                    }
                }
            }
            break;
        }
        
        case OPTIMIZING:
        {
            motors.stopBrake();
            robotRunning = false;
            
            unsigned long mappingTime = (millis() - mappingStartTime) / 1000;

            if (client && client.connected()){
                client.println("\n╔════════════════════════════════════════╗");
                client.println("║  RUN 1 COMPLETE - OPTIMIZING PATH    ║");
                client.println("╚════════════════════════════════════════╝");
                client.print("Mapping time: ");
                client.print(mappingTime);
                client.println(" seconds");
                client.print("Raw Path: ");
                client.print(rawPath);
                client.print(" (");
                client.print(rawPath.length());
                client.println(" moves)");
                // Serial.print("Average segment: ");
                // Serial.print(avgSegmentLength, 1);
                // Serial.println(" ticks");
            }
            
            if (rawPath.length() == 0) {
                if (client && client.connected()) client.println("❌ ERROR: No path recorded!");
                currentState = FINISHED;
                break;
            }
            
            // copy path
            optimizedPath = rawPath;
            optimizedPathLength = rawPath.length();
            
            // Copy segments
            for (int i = 0; i < pathIndex && i < MAX_PATH_LENGTH; i++) {
                optimizedSegments[i] = pathSegments[i];
            }
            
            if (client && client.connected()) client.println("\nOptimizing path...");
            
            // === Path Optimization with multiple iterations ===
            int iterations = 0;
            int consecutiveNoChange = 0;
            const int MAX_ITERATIONS = 50;
            const int MAX_NO_CHANGE = 3;
            
            while(iterations < MAX_ITERATIONS && consecutiveNoChange < MAX_NO_CHANGE) {
                int oldLength = optimizedPathLength;
                optimizer.optimize(optimizedPath, optimizedSegments, optimizedPathLength);
                
                if(oldLength == optimizedPathLength) {
                    consecutiveNoChange++;
                }
                else {
                    consecutiveNoChange = 0;
                }
                iterations++;
                delay(1);
            }
            
            if(consecutiveNoChange >= MAX_NO_CHANGE) {
                Serial.println("  (converged - no further improvements)");
            }
            
            // Serial.println("\n╔════════════════════════════════════════╗");
            // Serial.print("║  Final Path: ");
            // Serial.print(optimizedPath);
            // for(int i = optimizedPath.length(); i < 25; i++) Serial.print(" ");
            // Serial.println("║");
            // Serial.print("║  Length: ");
            // Serial.print(optimizedPath.length());
            // Serial.print(" moves");
            // for(int i = String(optimizedPath.length()).length(); i < 20; i++) Serial.print(" ");
            // Serial.println("║");
            // Serial.print("║  Saved: ");
            // Serial.print(rawPath.length() - optimizedPath.length());
            // Serial.print(" moves");
            // for(int i = String(rawPath.length() - optimizedPath.length()).length(); i < 21; i++) Serial.print(" ");
            // Serial.println("║");
            // Serial.println("╚════════════════════════════════════════╝\n");
            
            if (client && client.connected()) {
                client.println("\n╔════════════════════════════════════════╗");
                client.println("║      PATH OPTIMIZED!                   ║");
                client.println("╚════════════════════════════════════════╝");
                client.print("Raw: ");
                client.println(rawPath);
                client.print("Optimized: ");
                client.println(optimizedPath);
                client.print("Saved ");
                client.print(rawPath.length() - optimizedPath.length());
                client.println(" moves!");
            }
            currentState = WAIT_FOR_RUN_2;
            solvePathIndex = 0;
            break;
        }
        
        case WAIT_FOR_RUN_2:
        {

            // digitalWrite(RGB_PIN_R, HIGH);
            // digitalWrite(RGB_PIN_B, LOW);
            // digitalWrite(RGB_PIN_G, LOW);

            if (digitalRead(USER_BUTTON) == LOW || (robotRunning && optimizedPath.length() > 0)) {
                delay(50);
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║     RUN 2: SOLVING STARTED            ║");
                    Serial.println("╚════════════════════════════════════════╝\n");
                    
                    if (client && client.connected()) {
                        client.println("\n>>> RUN 2: SOLVING STARTED!");
                        client.print("Following optimized path: ");
                        client.println(optimizedPath);
                    }
                    
                    currentState = SOLVING;
                    robotRunning = true;
                    solvePathIndex = 0;
                    solveState = SOLVE_TURN;
                    solvingStartTime = millis();
                    lastError = 0;
                    integral = 0;
                    motors.clearEncoders();
                    
                    while(digitalRead(USER_BUTTON) == LOW) delay(1);

                    // digitalWrite(RGB_PIN_R, LOW);
                    // digitalWrite(RGB_PIN_B, HIGH);
                    // digitalWrite(RGB_PIN_G, HIGH);
                }
            }
            break;
        }
        
        case SOLVING:
        {
            if (!robotRunning) {
                motors.stopBrake();
                break;
            }
            if (optimizedPath.isEmpty() || optimizedPathLength == 0) {
                Serial.println("ERROR: Missing optimized path!");
                currentState = FINISHED;
                return;
            }
            
            if (solveState == SOLVE_TURN) {
                if (solvePathIndex >= optimizedPathLength) {
                    // Finished all turns - final segment
                    solveState = SOLVE_FINAL_RUN;
                    Serial.println("→ Final segment to finish");
                }
                else {
                    char turn = optimizedPath[solvePathIndex];
                    
                    if (turn == 'L') {
                        if (client && client.connected()) client.println("LEFT turn");
                        motors.turn_90_left_smart(sensors);
                    }
                    else if (turn == 'R') {
                        if (client && client.connected()) client.println("RIGHT turn");
                        motors.turn_90_right_smart(sensors);
                    }
                    else {
                        if (client && client.connected()) client.println("STRAIGHT");
                        // No turn needed for 'S'
                    }
                    
                    if (solvePathIndex >= optimizedPathLength - 1) {
                        solveState = SOLVE_FINAL_RUN;
                    }
                    else {
                        motors.clearEncoders();
                        solveState = SOLVE_FAST_RUN;
                    }
                    lastError = 0;
                    integral = 0;
                    delay(dl3);
                }

            }
            else if (solveState == SOLVE_FAST_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex];
                
                if (currentTicks < (targetTicks - SLOWDOWN_TICKS)) {
                    runPID(highSpeed);  // GO FAST!
                }
                else {
                    solveState = SOLVE_SLOW_RUN;
                }
                PathOptions pathsA = sensors.getAvailablePaths();
                
                // Count available pathsA
                int pathCounter = 0;
                if (pathsA.left) pathCounter++;
                if (pathsA.right) pathCounter++;
                if (pathsA.straight) pathCounter++;
                
                // Is this a junction?  (more than just straight OR only left/right)
                bool isJunction = (pathCounter > 1) || (pathCounter == 1 && !pathsA.straight);
                if(isJunction){
                    motors.stopBrake();
                    solveState = SOLVE_SLOW_RUN;
                    delay(dl4);
                }
            }
            else if (solveState == SOLVE_SLOW_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex];
                
                motors.moveForward(SLOWDOWN_TICKS);  // Slow down for accuracy

                motors.stopBrake();
                solvePathIndex++;
                solveState = SOLVE_TURN;
                delay(dl5);
                
            }
            else if (solveState == SOLVE_FINAL_RUN) {
                runPID(baseSpeed);
                
                // Check for finish (line end)
                if (sensors.isEndPoint()) {
                    motors.moveForward(250);
                    motors.stopBrake();
                    robotRunning = false;
                    
                    unsigned long solvingTime = (millis() - solvingStartTime) / 1000;
                    
                    if (client && client.connected()) {
                        client.println("\n╔════════════════════════════════════════╗");
                        client.println("║      🏆  MAZE SOLVED!  🏆              ║");
                        client.println("╚════════════════════════════════════════╝");
                        client.print("Time: ");
                        client.print(solvingTime);
                        client.println("s");
                    }
                    
                    currentState = FINISHED;
                }
            }
            break;
        }
            
        case FINISHED:
        {

            // digitalWrite(RGB_PIN_R, HIGH);
            // digitalWrite(RGB_PIN_B, LOW);
            // digitalWrite(RGB_PIN_G, LOW);

            // Victory blink
            int i = 0;
            while(i < 200){
            digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
            delay(30);
            digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
            delay(20);
            i+=50;
            }

            break;
        }
        
        case CALIBRATING:
            break;
    }
}

// === PID Line Following ===
void runPID(int speed, float correction_multiplier) {
    // Get current error from sensors
    float error = sensors.getLineError();
    
    // PID calculations
    float P = Kp * error;
    
    integral += error;
    // Anti-windup: constrain integral
    integral = constrain(integral, -maxIntegral, maxIntegral);
    float I = Ki * integral;
    
    float D = Kd * (error - lastError);
    
    // Total correction
    float correction = P + I + D;
    
    // Constrain correction
    correction = constrain(correction, -speed, speed) * correction_multiplier;
    
    // Apply to motors
    int leftSpeed = speed - (int)correction;
    int rightSpeed = speed + (int)correction;
    
    // Constrain motor speeds
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    
    // Save for next iteration
    lastError = error;
}

String junctionTypeToString(JunctionType type) {
    switch(type) {
        case JUNCTION_T_LEFT: return "T-Left ├";
        case JUNCTION_T_RIGHT: return "T-Right ┤";
        case JUNCTION_T_BOTH: return "T-Both ┬";
        case JUNCTION_CROSS: return "Cross ┼";
        case JUNCTION_90_LEFT: return "90° Left └";
        case JUNCTION_90_RIGHT: return "90° Right ┘";
        case JUNCTION_DEAD_END: return "Dead End";
        default: return "Straight";
    }
}

void setupWiFi() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║              WiFi Setup                ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    // ★★★ CRITICAL: Disconnect any previous connection ★★★
    WiFi.disconnect(true);
    delay(100);
    
    // ★★★ Set WiFi mode explicitly ★★★
    WiFi.mode(WIFI_STA);
    delay(100);
    
    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {  // ★ 40 attempts (20 sec)
        delay(500);
        Serial.print(".");
        
        // Print diagnostic status every 10 attempts
        if (attempts % 10 == 9) {
            Serial.print(" [Status: ");
            switch(WiFi.status()) {
                case WL_IDLE_STATUS:    Serial.print("IDLE"); break;
                case WL_NO_SSID_AVAIL:  Serial.print("NO SSID"); break;
                case WL_SCAN_COMPLETED: Serial.print("SCAN DONE"); break;
                case WL_CONNECT_FAILED: Serial.print("FAILED"); break;
                case WL_CONNECTION_LOST: Serial.print("LOST"); break;
                case WL_DISCONNECTED:   Serial.print("DISCONNECTED"); break;
                default: Serial.print(WiFi.status());
            }
            Serial.println("]");
        }
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi Connected!");
        Serial.print("✓ IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("✓ Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("Connect: telnet ");
        Serial.println(WiFi.localIP());
        server.begin();
        Serial.println("✓ Telnet Server Started on port 23");
    } else {
        Serial.println("\n❌ WiFi Connection Failed!");
        Serial.println("  Troubleshooting:");
        Serial.println("  1. Check SSID and password");
        Serial.println("  2. Ensure phone hotspot is ON");
        Serial.println("  3. Move closer to WiFi source");
        
        // ★★★ Scan for available networks for debugging ★★★
        Serial.println("\n  Scanning for available networks...");
        int n = WiFi.scanNetworks();
        if (n == 0) {
            Serial.println("  No networks found!");
        } else {
            Serial.printf("  Found %d networks:\n", n);
            for (int i = 0; i < n && i < 10; i++) {
                Serial.printf("    %d: %s (%d dBm)\n", i+1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
            }
        }
    }
}

void handleWiFiClient() {
    // ★★★ Properly detect incoming connections ★★★
    if (server.hasClient()) {
        if (!client || !client.connected()) {
            if (client) client.stop();  // Clean up old connection
            client = server.available();
            if (client) {
                Serial.println("✓ Telnet client connected");
                client.println("╔════════════════════════════════════════╗");
                client.println("║  Mesmerize Maze Solver Console        ║");
                client.println("║  V3.1 - Corrected Debouncing          ║");
                client.println("╚════════════════════════════════════════╝");
                printMenu();
            }
        }
    }
    
    if (client && client.connected() && client.available()) {
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            Serial.print("[Telnet] ");
            Serial.println(cmd);
            processCommand(cmd);
        }
    }
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd == "START" || cmd == "GO") {
        if (currentState == WAIT_FOR_RUN_1 || currentState == WAIT_FOR_RUN_2) {
            robotRunning = true;
            client.println("✓ STARTING...");
        } else {
            client.println("❌ Not in waiting state");
        }
    }
    else if (cmd == "STOP" || cmd == "S") {
        robotRunning = false;
        motors.stopBrake();
        client.println("✓ STOPPED");
    }
    else if (cmd == "RESET" || cmd == "R") {
        currentState = WAIT_FOR_RUN_1;
        robotRunning = false;
        motors.stopBrake();
        lastError = 0;
        integral = 0;
        junctionCount = 0;
        pathIndex = 0;
        rawPath = "";
        optimizedPath = "";
        client.println("✓ RESET - Ready for Run 1");
    }
    else if (cmd == "STATUS" || cmd == "ST") {
        printStatus();
    }
    else if (cmd == "PATH") {
        client.println("\n=== Path Info ===");
        client.print("Raw: ");
        client.print(rawPath);
        client.print(" (");
        client.print(rawPath.length());
        client.println(" moves)");
        client.print("Optimized: ");
        client.print(optimizedPath);
        client.print(" (");
        client.print(optimizedPath.length());
        client.println(" moves)");
        if (rawPath.length() > 0 && optimizedPath.length() > 0) {
            client.print("Saved: ");
            client.print(rawPath.length() - optimizedPath.length());
            client.println(" moves");
        }
        client.println("=================\n");
    }
    else if (cmd.startsWith("DEBOUNCE ")) {
        int deb = cmd.substring(9).toInt();
        junctionDebounce = deb;
        client.println("\n=== Debounce Info ===");
        client.printf("Base Debounce: %lums\n", junctionDebounce);
        client.printf("Current Speed: %d\n", baseSpeed);
        client.printf("Dynamic Debounce: %lums\n", getDynamicDebounce());
        client.println("Logic: Slower=Longer, Faster=Shorter");
        client.println("=====================\n");
    }
    else if (cmd == "HELP" || cmd == "H") {
        printMenu();
    }
    
    // === PID TUNING ===
    else if (cmd.startsWith("KP ")) {
        Kp = cmd.substring(3).toFloat();
        client.printf("✓ Kp = %.2f\n", Kp);
        Serial.printf("WiFi: Kp = %.2f\n", Kp);
    }
    else if (cmd.startsWith("KI ")) {
        Ki = cmd.substring(3).toFloat();
        client.printf("✓ Ki = %.3f\n", Ki);
        Serial.printf("WiFi: Ki = %.3f\n", Ki);
    }
    else if (cmd.startsWith("KD ")) {
        Kd = cmd.substring(3).toFloat();
        client.printf("✓ Kd = %.2f\n", Kd);
        Serial.printf("WiFi: Kd = %.2f\n", Kd);
    }
    else if (cmd.startsWith("TUNE ")) {
        int s1 = cmd.indexOf(' ', 5);
        int s2 = cmd.indexOf(' ', s1 + 1);
        if (s1 > 0 && s2 > 0) {
            Kp = cmd.substring(5, s1).toFloat();
            Ki = cmd.substring(s1 + 1, s2).toFloat();
            Kd = cmd.substring(s2 + 1).toFloat();
            client.printf("✓ PID: Kp=%.1f Ki=%.2f Kd=%.1f\n", Kp, Ki, Kd);
            integral = 0;  // Reset integral when tuning
        }
    }
    
    // === SPEED TUNING ===
    else if (cmd.startsWith("SPEED ")) {
        baseSpeed = cmd.substring(6).toInt();
        baseSpeed = constrain(baseSpeed, 50, maxSpeed);
        client.printf("✓ Base Speed = %d (Dynamic DB now: %lums)\n", baseSpeed, getDynamicDebounce());
    }
    else if (cmd.startsWith("HIGHSPEED ")) {
        highSpeed = cmd.substring(10).toInt();
        highSpeed = constrain(highSpeed, 50, maxSpeed);
        client.printf("✓ High Speed = %d\n", highSpeed);
    }
    else if(cmd.startsWith("MAXSPEED ")){
        maxSpeed = cmd.substring(8).toInt();
        client.printf("Max speed = %d\n", maxSpeed);
    }
    else if (cmd.startsWith("JUNCTIONDB ")) {
        junctionDebounce = cmd.substring(11).toInt();
        junctionDebounce = constrain(junctionDebounce, 100, 1000);
        client.printf("✓ Base Junction Debounce = %lums (Dynamic DB now: %lums)\n", 
                    junctionDebounce, getDynamicDebounce());
    }
    
    // === MOTOR TICK TUNING ===
    else if (cmd.startsWith("CENTER ")) {
        int ticks = cmd.substring(7).toInt();
        Motors::updateCenterTicks(ticks);
        client.printf("✓ Center Ticks = %d\n", ticks);
        Serial.printf("WiFi: Center Ticks = %d\n", ticks);
    }
    else if (cmd.startsWith("TURN90 ")) {
        int ticks = cmd.substring(7).toInt();
        Motors::updateTurn_90_Ticks(ticks);
        client.printf("✓ Turn 90° Ticks = %d\n", ticks);
    }
    else if (cmd.startsWith("MOTORS ")) {
        int s1 = cmd.indexOf(' ', 7);
        if (s1 > 0) {
            int centerTicks = cmd.substring(7, s1).toInt();
            int turn90Ticks = cmd.substring(s1 + 1).toInt();
            Motors::updateCenterTicks(centerTicks);
            Motors::updateTurn_90_Ticks(turn90Ticks);
            client.printf("✓ Motors: Center=%d Turn90=%d\n", centerTicks, turn90Ticks);
            Serial.printf("WiFi: Motors: Center=%d Turn90=%d\n", centerTicks, turn90Ticks);
        }
    }
    else if (cmd.startsWith("TURN180 ")){
        int ticks = cmd.substring(8).toInt();
        Motors::updateTurn_180_Ticks(ticks);
        client.printf("✓ Turn 180° Ticks = %d\n", ticks);
    }
    // NEW: Min turn percent tuning
    else if (cmd.startsWith("MINTP ")) {
        int percent = cmd.substring(6).toInt();
        Motors::updateMinTurnPercent(percent);
        client.printf("✓ Min Turn Percent = %d%%\n", MIN_TURN_PERCENT);
    }
    // add state change
    else if (cmd.startsWith("WAIT")){
        currentState = WAIT_FOR_RUN_2;
    }

    //addition for turnings speed controll
    else if (cmd.startsWith("TS ")) {
        int speed = cmd.substring(3).toInt();
        Motors::updateSpeeds(150, speed, 200);
        client.printf("✓ Turn 90° Ticks = %d\n", speed);
    }
    // slowdown ticks
    else if (cmd.startsWith("SDT ")) {
        int ticks = cmd.substring(4).toInt();
        SLOWDOWN_TICKS = ticks;
        client.printf("✓ Slow_Down Ticks = %d\n", ticks);
    }

    //solving state slow down
    else if(cmd.startsWith("SLOW ")){
        int ticks = cmd.substring(5).toInt(); 
        SLOWDOWN_TICKS = ticks;
        client.printf("SLOWDOWN_TICKS = %d\n", SLOWDOWN_TICKS);
    }
    

    //addition for delays
    else if (cmd.startsWith("DBC ")){
        int dl = cmd.substring(4).toInt(); 
        delayBeforeCenter = dl;
        client.printf("delay before center: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DAC ")){
        int dl = cmd.substring(4).toInt(); 
        delayAfterCenter = dl;
        client.printf("delay after center: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DL1 ")){
        int dl = cmd.substring(3).toInt(); 
        dl1 = dl;
        client.printf("changed dl1: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DL2 ")){
        int dl = cmd.substring(4).toInt(); 
        dl2 = dl;
        client.printf("changed dl2: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DL3 ")){
        int dl = cmd.substring(4).toInt(); 
        dl3 = dl;
        client.printf("changed dl3: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DL4 ")){
        int dl = cmd.substring(4).toInt(); 
        dl4 = dl;
        client.printf("changed dl4: %d", dl);
        client.println();
    }
    else if (cmd.startsWith("DL5 ")){
        int dl = cmd.substring(4).toInt(); 
        dl5 = dl;
        client.printf("changed dl4: %d", dl);
        client.println();
    }

    // pid alignment correction

    // else if (cmd.startsWith("AS ")){
    //     int speed = cmd.substring(3).toInt(); 
    //     adjusted_speed = speed;
    //     client.printf("changed adjusted speed: %d", speed);
    //     client.println();
    // }
    // else if (cmd.startsWith("ACT ")){
    //     int t = cmd.substring(4).toInt(); 
    //     align_corr_time = t;
    //     client.printf("changed alignment correction time: %d", t);
    //     client.println();
    // }
    // else if (cmd.startsWith("CM ")){
    //     float factor = cmd.substring(3).toInt(); 
    //     correction_multiplier = factor;
    //     client.printf("changed correction multiplier: %d", factor);
    //     client.println();
    // }

    // === TESTING ===
    else if (cmd == "TEST" || cmd == "T") {
        bool sensors_arr[8];
        sensors.getSensorArray(sensors_arr);
        
        client.println("\n=== Sensor Test ===");
        client.print("Pattern: [");
        for (int i = 0; i < 8; i++) {
            client.print(sensors_arr[i] ?  "█" : "·");
        }
        client.println("]");
        client.printf("Error: %.2f\n", sensors.getLineError());
        client.printf("Position: %.2f\n", sensors.getPosition());
        client.printf("Active: %d sensors\n", sensors.getActiveSensorCount());
        client.printf("On Line: %s\n", sensors.onLine() ? "YES" : "NO");
        client.printf("Finish: %s\n", sensors.isEndPoint() ? "YES" : "NO");
        client.println("==================\n");
    }
    else if (cmd == "ENCTEST") {
    motors.clearEncoders();
    motors.setSpeeds(100, 100);  // Both forward
    delay(1000);
    motors.stopBrake();
    
    long left = motors.getLeftCount();
    long right = motors.getRightCount();
    
    client.printf("Left encoder:  %ld\n", left);
    client.printf("Right encoder: %ld\n", right);
    client.printf("Difference:    %ld (%.1f%%)\n", 
                abs(left - right), 
                100.0 * abs(left - right) / max(left, right));
    
    // Run test 5 times
    for (int t = 0; t < 5; t++) {
        motors.clearEncoders();
        motors.setSpeeds(100, 100);
        delay(500);
        motors.stopBrake();
        delay(100);
        client.printf("Trial %d: L=%ld R=%ld diff=%ld\n", 
                    t, motors.getLeftCount(), motors.getRightCount(),
                    abs(motors.getLeftCount() - motors.getRightCount()));
    }
}

    //sensing
    else if (cmd == "CAL") {
        sensors.printCalibrationToClient(client);
    }
    else if (cmd == "RAW") {
        uint16_t rawValues[8];
        bool digital[8];
        sensors.readRaw(rawValues);
        sensors.readDigital(digital);
        
        client.println("\n=== Raw Sensor Readings ===");
        client.print("Digital: [");
        for (int i = 0; i < 8; i++) {
            client.print(digital[i] ? "█" : "·");
        }
        client.println("]");
        client.println("Sensor | Raw    | Detected");
        client.println("-------|--------|----------");
        for (int i = 0; i < 8; i++) {
            client.printf("  S%-2d  | %-6d | %s\n", i+1, rawValues[i], digital[i] ? "LINE" : "-");
        }
        client.printf("Sensitivity: %.2f\n", sensors.getSensitivity());
        client.println("===========================\n");
    }
    else if (cmd.startsWith("SENS ")) {
        float sens = cmd.substring(5).toFloat();
        sensors.setSensitivity(sens);
        client.printf("✓ Sensitivity = %.2f\n", sensors.getSensitivity());
        client.println("  0.00 = most sensitive (default)");
        client.println("  1.00 = least sensitive (only strong white)");
        // Show what happened to thresholds
        sensors.printCalibrationToClient(client);
    }

    //pid
    else if (cmd == "PID") {
        client.println("\n=== PID Values ===");
        client.printf("Kp = %.2f\n", Kp);
        client.printf("Ki = %.3f\n", Ki);
        client.printf("Kd = %.2f\n", Kd);
        client.printf("Last Error = %.2f\n", lastError);
        client.printf("Integral = %.2f\n", integral);
        client.println("==================\n");
    }
    else {
        client.println("❌ Unknown.Type HELP");
    }
}

void printMenu() {
    client.println("\n=== Commands ===");
    client.println("START / GO - Start robot");
    client.println("STOP / S - Stop robot");
    client.println("RESET / R - Reset to Run 1");
    client.println("HELP / H - Show this menu");
    client.println("STATUS / ST - Show status");
    client.println("PATH - Show path info");
    client.println("DEBOUNCE - Show debounce info");
    client.println("TEST - Test sensors");
    client.println("=== Sensitivity ===");
    client.println("SENS <0.0-1.0> - Set sensitivity (0=max, 1=min)");
    client.println("RAW  - Show raw sensor readings");
    client.println("CAL  - Show calibration thresholds");
    client.println("PID - Show PID values");
    client.println("");
    client.println("=== PID Tuning ===");
    client.println("KP <val> - Set proportional gain");
    client.println("KI <val> - Set integral gain");
    client.println("KD <val> - Set derivative gain");
    client.println("TUNE <kp> <ki> <kd> - Set all PID");
    client.println("");
    client.println("=== Speed Settings ===");
    client.println("SPEED <val> - Base speed");
    client.println("HIGHSPEED <val> - High speed for Run 2");
    client.println("");
    client.println("=== Junction Settings ===");
    client.println("JUNCTIONDB <ms> - Base junction debounce");
    client.println("");
    client.println("=== Motor Ticks ===");
    client.println("CENTER <ticks> - Set ticks to center");
    client.println("TURN90 <ticks> - Set 90° turn ticks");
    client.println("MOTORS <center> <turn90> - Set both");
    client.println("TURN180 <ticks> - Set 180° turn ticks");
    client.println("MINTP <percent> - Min % before sensor check (30-95)");
    client.println("================\n");
}
void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    switch(currentState) {
        case WAIT_FOR_RUN_1: client.println("WAIT_RUN_1"); break;
        case MAPPING: client.println("MAPPING"); break;
        case OPTIMIZING: client.println("OPTIMIZING"); break;
        case WAIT_FOR_RUN_2: client.println("WAIT_RUN_2"); break;
        case SOLVING: client.println("SOLVING"); break;
        case FINISHED: client.println("FINISHED"); break;
        default: client.println("CALIBRATING");
    }
    client.printf("PID: Kp=%.1f Ki=%.2f Kd=%.1f\n", Kp, Ki, Kd);
    client.printf("Speed: Base=%d High=%d\n", baseSpeed, highSpeed);
    client.printf("Junction Debounce: Base=%lums Dynamic=%lums\n", 
                junctionDebounce, getDynamicDebounce());
    client.printf("Motor Ticks: Center=%d Turn90=%d Turn180=%d\n", TICKS_TO_CENTER, TICKS_FOR_90_DEG, TICKS_FOR_180_DEG);
    client.printf("Error: %.2f\n", sensors.getLineError());
    client.printf("Junctions: %d\n", junctionCount);
    client.printf("Path Index: %d\n", pathIndex);
    client.printf("On Line: %s\n", sensors.onLine() ? "YES" : "NO");
    client.printf("Paths: %d\n", rawPath);
    client.println("==============\n");
}