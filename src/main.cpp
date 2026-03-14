/*
 * main.cpp
 * Industrial-Grade Continuous PID Line Maze Solver
 *
 * Architecture:
 *   Outer loop (steering PID): line sensor error → ΔV differential
 *   Inner loop (velocity PID): per-wheel target mm/s → PWM
 *   Junction detection: distance-confirmed (no stopping)
 *   Path storage: LSRB with segment distances in mm
 *   Fast run: pure continuous control via odometry + saved path
 */

#include <Arduino.h>
#include <WiFi.h>
#include <QuickPID.h>
#include "Pins.h"
#include "Config.h"
#include "Sensors.h"
#include "Motors.h"
#include "Odometry.h"
#include "PathOptimization.h"

// =====================================================================
//  WiFi
// =====================================================================
WiFiServer server(TELNET_PORT);
WiFiClient client;

// =====================================================================
//  Global Objects
// =====================================================================
Sensors          sensors;
Motors           motors;
Odometry         odometry;
PathOptimization optimizer;

// =====================================================================
//  Steering PID (outer loop)
//  Input:  line sensor error (-3500 to +3500)
//  Output: differential velocity in mm/s (ΔV)
// =====================================================================
float steerInput    = 0.0f;   // sensor error
float steerOutput   = 0.0f;   // ΔV
float steerSetpoint = 0.0f;   // normally 0 (centered on line)

QuickPID steerPID(&steerInput, &steerOutput, &steerSetpoint,
                  STEER_KP_DEFAULT, STEER_KI_DEFAULT, STEER_KD_DEFAULT,
                  QuickPID::Action::direct);

// =====================================================================
//  Speed Settings (WiFi-tunable)
// =====================================================================
float currentCruiseSpeed = DEFAULT_CRUISE_SPEED_MMS;

// =====================================================================
//  Junction Detection State
// =====================================================================
float junctionConfirmMM   = JUNCTION_CONFIRM_MM_DEFAULT;  // WiFi-tunable
float junctionMinSpacing  = JUNCTION_MIN_SPACING_MM;

bool  leftCandidateActive  = false;
bool  rightCandidateActive = false;
float candidateStartMM     = 0.0f;
float lastJunctionMM       = 0.0f;  // distance at last confirmed junction

// Tracking detected branches during confirmation window
bool  confirmedLeft   = false;
bool  confirmedRight  = false;
bool  confirmedStraight = false;

// =====================================================================
//  Turn Execution State
// =====================================================================
enum TurnMode {
    TURN_NONE,       // Normal line following
    TURN_HEADING,    // Executing a heading-based turn (odometry)
    TURN_REACQUIRE   // Searching for line after turn
};
TurnMode turnMode = TURN_NONE;
float    targetHeading    = 0.0f;   // target heading in radians
float    headingTolerance = 0.15f;  // ~8.6 degrees  (WiFi-tunable)
unsigned long turnStartMs = 0;
const unsigned long TURN_TIMEOUT_MS = 3000;  // safety timeout

// =====================================================================
//  Path Storage
// =====================================================================
String rawPath = "";
float  pathSegments[MAX_PATH_LENGTH];
int pathIndex = 0;

String optimizedPath = "";
float  optimizedSegments[MAX_PATH_LENGTH];
int    optimizedPathLength = 0;
int    solvePathIndex = 0;

int    junctionCount = 0;

// =====================================================================
//  Dead-end Detection
// =====================================================================
unsigned long lineEndStartMs = 0;

// =====================================================================
//  State Machine
// =====================================================================
enum State {
    CALIBRATING,
    WAIT_FOR_RUN_1,
    MAPPING,
    OPTIMIZING,
    WAIT_FOR_RUN_2,
    SOLVING,
    FINISHED
};
State currentState = CALIBRATING;
bool  robotRunning = false;

// =====================================================================
//  Timing
// =====================================================================
unsigned long mappingStartTime  = 0;
unsigned long solvingStartTime  = 0;
unsigned long lastWiFiUpdate    = 0;
unsigned long lastDebugPrint    = 0;
unsigned long lastControlUs     = 0;
unsigned long buttonPressStart  = 0;

// =====================================================================
//  Function Declarations
// =====================================================================
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();

void runControlLoop();
void handleMappingJunction();
void handleDeadEnd();
void executeTurn(char direction);
void resetControlState();
char decideLSRB(bool left, bool straight, bool right);

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  Line Maze Solver — Cascade PID       ║");
    Serial.println("║  Continuous Motion / No Junction Stops ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    pinMode(ONBOARD_LED, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);

    motors.setup();

    Serial.println("Waiting for button press to calibrate...");
    setupWiFi();
    while (true) {
        handleWiFiClient();
        yield();
        if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
            delay(50);
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) break;
        }
    }
    delay(2000);

    // Calibrate sensors while spinning
    motors.rotate();
    sensors.setup();
    motors.stopBrake();

    // Initialize odometry with encoder pointers
    odometry.begin(motors.getLeftEncoder(), motors.getRightEncoder());

    // Configure steering PID
    steerPID.SetOutputLimits(-maxSpeedMMS, maxSpeedMMS);
    steerPID.SetSampleTimeUs(STEER_INTERVAL_US);
    steerPID.SetMode(QuickPID::Control::automatic);

    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  Configuration                        ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf( "║  Wheel Ø: %.1f mm  Base: %.1f mm      ║\n", WHEEL_DIAMETER_MM, WHEEL_BASE_MM);
    Serial.printf( "║  mm/tick: %.4f   ticks/mm: %.3f      ║\n", MM_PER_TICK, TICKS_PER_MM);
    Serial.printf( "║  Sensor offset: %.1f mm               ║\n", SENSOR_OFFSET_MM);
    Serial.printf( "║  Cruise: %.0f mm/s  Max: %.0f mm/s    ║\n", currentCruiseSpeed, maxSpeedMMS);
    Serial.printf( "║  Steer PID: Kp=%.5f Ki=%.5f Kd=%.5f  ║\n", STEER_KP_DEFAULT, STEER_KI_DEFAULT, STEER_KD_DEFAULT);
    Serial.printf( "║  Junc confirm: %.1f mm                ║\n", junctionConfirmMM);
    Serial.println("╚════════════════════════════════════════╝\n");

    currentState = WAIT_FOR_RUN_1;
    lastControlUs = micros();
    Serial.println("✓ Ready for Run 1 (Mapping)");
    Serial.println("Press button or type START via WiFi\n");
}

// =====================================================================
//  MAIN CONTROL LOOP
//  Called every loop() iteration.  Handles cascade PID and motor output.
// =====================================================================
void runControlLoop() {
    // 1. Update odometry
    odometry.update();

    // 2. Update velocity PID (inner loop — runs every call)
    motors.updateVelocityPID();

    // 3. Steering logic depends on turn mode
    if (turnMode == TURN_NONE) {
        // Normal line following — steering PID tracks line error
        steerInput   = sensors.getLineError();
        steerSetpoint = 0.0f;
        steerPID.Compute();

        float leftTarget  = currentCruiseSpeed - steerOutput;
        float rightTarget = currentCruiseSpeed + steerOutput;
        motors.setTargetVelocities(leftTarget, rightTarget);
    }
    else if (turnMode == TURN_HEADING) {
        // Executing a heading-based turn
        float headingError = targetHeading - odometry.getHeading();

        // Normalize to [-π, π]
        while (headingError >  (float)M_PI) headingError -= 2.0f * (float)M_PI;
        while (headingError < -(float)M_PI) headingError += 2.0f * (float)M_PI;

        if (fabsf(headingError) < headingTolerance) {
            // Turn complete — switch to line reacquisition
            turnMode = TURN_REACQUIRE;
        } else {
            // Proportional heading control with some forward speed
            float turnRate = constrain(headingError * 3.0f, -1.0f, 1.0f);  // normalized -1..1
            float fwd   = turnSpeedMMS * 0.3f;   // slow forward during turn
            float diff   = turnSpeedMMS * turnRate;

            motors.setTargetVelocities(fwd - diff, fwd + diff);
        }

        // Safety timeout
        if (millis() - turnStartMs > TURN_TIMEOUT_MS) {
            turnMode = TURN_REACQUIRE;
        }
    }
    else if (turnMode == TURN_REACQUIRE) {
        // Line reacquisition: follow the line error but at reduced speed
        if (sensors.onLine()) {
            // Line found — resume normal following
            turnMode = TURN_NONE;
            steerOutput = 0.0f;
        } else {
            // Keep rotating in the direction of the last turn to find line
            float headingError = targetHeading - odometry.getHeading();
            while (headingError >  (float)M_PI) headingError -= 2.0f * (float)M_PI;
            while (headingError < -(float)M_PI) headingError += 2.0f * (float)M_PI;

            float turnDir = (headingError >= 0) ? 1.0f : -1.0f;
            motors.setTargetVelocities(-turnSpeedMMS * 0.5f * turnDir,
                                        turnSpeedMMS * 0.5f * turnDir);

            // Timeout: if we've been searching too long, just go forward
            if (millis() - turnStartMs > TURN_TIMEOUT_MS + 1000) {
                turnMode = TURN_NONE;
            }
        }
    }
}

// =====================================================================
//  TURN EXECUTION
// =====================================================================
void executeTurn(char direction) {
    float currentHeading = odometry.getHeading();

    switch (direction) {
        case 'L':
            targetHeading = currentHeading + TURN_90_RAD;
            break;
        case 'R':
            targetHeading = currentHeading - TURN_90_RAD;
            break;
        case 'B':
            targetHeading = currentHeading + TURN_180_RAD;
            break;
        case 'S':
            // Straight — no turn needed
            return;
        default:
            return;
    }

    // Normalize target heading
    while (targetHeading >  (float)M_PI) targetHeading -= 2.0f * (float)M_PI;
    while (targetHeading < -(float)M_PI) targetHeading += 2.0f * (float)M_PI;

    turnMode = TURN_HEADING;
    turnStartMs = millis();
}

// =====================================================================
//  LSRB DECISION
// =====================================================================
char decideLSRB(bool left, bool straight, bool right) {
    if (left)     return 'L';
    if (straight) return 'S';
    if (right)    return 'R';
    return 'B';  // dead end
}

// =====================================================================
//  RESET CONTROL STATE
// =====================================================================
void resetControlState() {
    steerInput = steerOutput = steerSetpoint = 0.0f;
    turnMode = TURN_NONE;
    leftCandidateActive = rightCandidateActive = false;
    confirmedLeft = confirmedRight = confirmedStraight = false;
    lineEndStartMs = 0;
}

// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {
    yield();
    handleWiFiClient();

    // === WiFi Telemetry ===
    if (millis() - lastWiFiUpdate > 1000 && client && client.connected()) {
        if (client.availableForWrite() > 100) {
            bool sv[8];
            sensors.getSensorArray(sv);
            client.print("S:[");
            for (int i = 0; i < 8; i++) client.print(sv[i] ? "█" : "·");
            client.printf("] Err:%.0f Spd:%.0f TM:%d | ",
                steerInput, currentCruiseSpeed, turnMode);

            switch (currentState) {
                case WAIT_FOR_RUN_1: client.print("WAIT_RUN1"); break;
                case MAPPING:        client.print("MAPPING"); break;
                case OPTIMIZING:     client.print("OPTIMIZING"); break;
                case WAIT_FOR_RUN_2: client.print("WAIT_RUN2"); break;
                case SOLVING:        client.print("SOLVING"); break;
                case FINISHED:       client.print("FINISHED"); break;
                default:             client.print("CALIBRATING");
            }

            if (currentState == MAPPING || currentState == SOLVING) {
                client.printf(" | J:%d Seg:%.0fmm", junctionCount, odometry.getSegmentMM());
                if (currentState == SOLVING && optimizedPathLength > 0) {
                    client.printf(" [%d/%d]", solvePathIndex, optimizedPathLength);
                }
            }
            client.println();
            client.flush();
        }
        lastWiFiUpdate = millis();
    }

    // === Emergency Stop (2s button hold) ===
    if (digitalRead(USER_BUTTON) == LOW) {
        if (buttonPressStart == 0) {
            buttonPressStart = millis();
        } else if (millis() - buttonPressStart > 2000) {
            motors.stopBrake();
            robotRunning = false;
            currentState = FINISHED;
            Serial.println("\n⚠️ EMERGENCY STOP!");
            if (client && client.connected()) client.println("⚠️ EMERGENCY STOP!");
            buttonPressStart = 0;
        }
    } else {
        buttonPressStart = 0;
    }

    // ================================================================
    //  STATE MACHINE
    // ================================================================
    switch (currentState) {

    // ----------------------------------------------------------------
    case WAIT_FOR_RUN_1:
    {
        if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
            delay(50);
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                if (client && client.connected())
                    client.println("\n>>> RUN 1: MAPPING STARTED!");

                currentState   = MAPPING;
                robotRunning   = true;
                pathIndex      = 0;
                rawPath        = "S";   // start with 'S' (start marker)
                junctionCount  = 0;
                mappingStartTime = millis();

                resetControlState();
                odometry.reset();
                motors.clearEncoders();
                odometry.begin(motors.getLeftEncoder(), motors.getRightEncoder());
                lastJunctionMM = 0.0f;

                while (digitalRead(USER_BUTTON) == LOW) delay(10);
            }
        }
        break;
    }

    // ----------------------------------------------------------------
    case MAPPING:
    {
        if (!robotRunning) {
            motors.stopBrake();
            currentState = FINISHED;
            break;
        }

        // Run cascade PID control loop
        runControlLoop();

        // === Distance-confirmed junction detection ===
        if (turnMode == TURN_NONE) {
            float currentDist = odometry.getSegmentMM();

            bool leftNow   = sensors.hasLeftBranch();
            bool rightNow  = sensors.hasRightBranch();
            bool straightNow = sensors.hasStraight();
            bool branchNow = leftNow || rightNow;

            // Min spacing check (don't double-count junctions)
            bool spacingOK = (currentDist - lastJunctionMM > junctionMinSpacing)
                          || (lastJunctionMM == 0.0f);

            if (branchNow && spacingOK) {
                if (!leftCandidateActive && !rightCandidateActive) {
                    // Start candidate confirmation window
                    leftCandidateActive  = leftNow;
                    rightCandidateActive = rightNow;
                    candidateStartMM     = currentDist;
                    confirmedLeft    = leftNow;
                    confirmedRight   = rightNow;
                    confirmedStraight = straightNow;
                } else {
                    // Accumulate detections during confirmation window
                    confirmedLeft    = confirmedLeft   || leftNow;
                    confirmedRight   = confirmedRight  || rightNow;
                    confirmedStraight = confirmedStraight || straightNow;

                    float traveled = currentDist - candidateStartMM;
                    if (traveled >= junctionConfirmMM) {
                        // ★ CONFIRMED JUNCTION ★
                        float segmentDist = candidateStartMM;  // distance to junction start

                        junctionCount++;

                        if (client && client.connected()) {
                            client.printf("J%d @ %.0fmm: L=%d S=%d R=%d\n",
                                junctionCount, segmentDist,
                                confirmedLeft, confirmedStraight, confirmedRight);
                        }

                        // Check for endpoint (all sensors on)
                        if (sensors.isEndPoint()) {
                            motors.stopBrake();
                            robotRunning = false;
                            unsigned long runTime = (millis() - mappingStartTime) / 1000;
                            if (client && client.connected()) {
                                client.println("\n🏆 MAPPING COMPLETE!");
                                client.printf("Time: %lus | Junctions: %d\n", runTime, junctionCount);
                            }
                            currentState = OPTIMIZING;
                            break;
                        }

                        // Save segment
                        if (pathIndex < MAX_PATH_LENGTH) {
                            pathSegments[pathIndex] = segmentDist;

                            // LSRB decision
                            char decision = decideLSRB(confirmedLeft, confirmedStraight, confirmedRight);
                            rawPath += decision;

                            if (client && client.connected()) {
                                client.printf("  → %c  (seg: %.0fmm)\n", decision, segmentDist);
                            }

                            pathIndex++;

                            // Execute turn
                            executeTurn(decision);

                            // Reset segment tracking
                            lastJunctionMM = currentDist;
                            odometry.resetSegment();
                        }

                        // Reset candidate state
                        leftCandidateActive = rightCandidateActive = false;
                        confirmedLeft = confirmedRight = confirmedStraight = false;
                    }
                }
            } else if (!branchNow && (leftCandidateActive || rightCandidateActive)) {
                // Branch disappeared before confirmation → was drift, reset
                leftCandidateActive = rightCandidateActive = false;
                confirmedLeft = confirmedRight = confirmedStraight = false;
            }

            // === Dead-end detection ===
            if (sensors.isLineEnd()) {
                if (lineEndStartMs == 0) {
                    lineEndStartMs = millis();
                } else if (millis() - lineEndStartMs > DEAD_END_CONFIRM_MS) {
                    // Confirmed dead end
                    float segmentDist = odometry.getSegmentMM();
                    junctionCount++;

                    if (client && client.connected()) {
                        client.printf("DEAD END @ %.0fmm\n", segmentDist);
                    }

                    if (pathIndex < MAX_PATH_LENGTH) {
                        pathSegments[pathIndex] = segmentDist;
                        rawPath += 'B';
                        pathIndex++;
                    }

                    executeTurn('B');
                    odometry.resetSegment();
                    lastJunctionMM = 0.0f;
                    lineEndStartMs = 0;
                    leftCandidateActive = rightCandidateActive = false;
                }
            } else {
                lineEndStartMs = 0;
            }
        }
        break;
    }

    // ----------------------------------------------------------------
    case OPTIMIZING:
    {
        motors.stopBrake();
        robotRunning = false;

        unsigned long mappingTime = (millis() - mappingStartTime) / 1000;

        if (client && client.connected()) {
            client.println("\n╔════════════════════════════════════════╗");
            client.println("║  RUN 1 COMPLETE — OPTIMIZING PATH     ║");
            client.println("╚════════════════════════════════════════╝");
            client.printf("Time: %lus\n", mappingTime);
            client.printf("Raw Path: %s (%d moves)\n", rawPath.c_str(), rawPath.length());
        }

        if (rawPath.length() == 0) {
            if (client && client.connected()) client.println("❌ ERROR: No path recorded!");
            currentState = FINISHED;
            break;
        }

        // Copy to optimized
        optimizedPath = rawPath;
        optimizedPathLength = rawPath.length();
        for (int i = 0; i < pathIndex && i < MAX_PATH_LENGTH; i++) {
            optimizedSegments[i] = pathSegments[i];
        }

        // Optimize with multiple iterations
        int iterations = 0;
        int noChange = 0;
        while (iterations < 50 && noChange < 3) {
            int oldLen = optimizedPathLength;
            optimizer.optimize(optimizedPath, optimizedSegments, optimizedPathLength);
            if (oldLen == optimizedPathLength) noChange++;
            else noChange = 0;
            iterations++;
            delay(1);
        }

        if (client && client.connected()) {
            client.println("\n╔════════════════════════════════════════╗");
            client.println("║      PATH OPTIMIZED!                  ║");
            client.println("╚════════════════════════════════════════╝");
            client.printf("Raw:       %s\n", rawPath.c_str());
            client.printf("Optimized: %s\n", optimizedPath.c_str());
            client.printf("Saved %d moves!\n", rawPath.length() - optimizedPath.length());

            client.println("\nSegment distances (mm):");
            for (int i = 0; i < optimizedPathLength; i++) {
                client.printf("  [%d] %c → %.0f mm\n", i, optimizedPath[i], optimizedSegments[i]);
            }
        }

        currentState = WAIT_FOR_RUN_2;
        solvePathIndex = 0;
        break;
    }

    // ----------------------------------------------------------------
    case WAIT_FOR_RUN_2:
    {
        if (digitalRead(USER_BUTTON) == LOW || (robotRunning && optimizedPath.length() > 0)) {
            delay(50);
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                if (client && client.connected()) {
                    client.println("\n>>> RUN 2: SOLVING STARTED!");
                    client.printf("Following path: %s\n", optimizedPath.c_str());
                }

                currentState     = SOLVING;
                robotRunning     = true;
                solvePathIndex   = 0;
                solvingStartTime = millis();

                resetControlState();
                odometry.reset();
                motors.clearEncoders();
                odometry.begin(motors.getLeftEncoder(), motors.getRightEncoder());

                while (digitalRead(USER_BUTTON) == LOW) delay(1);
            }
        }
        break;
    }

    // ----------------------------------------------------------------
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

        // Run cascade PID
        runControlLoop();

        // Check if we've finished all segments
        if (solvePathIndex >= optimizedPathLength) {
            // Final segment — run until endpoint
            if (sensors.isEndPoint()) {
                motors.stopBrake();
                robotRunning = false;
                unsigned long solveTime = (millis() - solvingStartTime) / 1000;
                if (client && client.connected()) {
                    client.println("\n╔════════════════════════════════════════╗");
                    client.println("║      🏆  MAZE SOLVED!  🏆             ║");
                    client.println("╚════════════════════════════════════════╝");
                    client.printf("Solve time: %lus\n", solveTime);
                }
                currentState = FINISHED;
            }
        }
        else if (turnMode == TURN_NONE) {
            // Check if we've traveled enough distance for this segment
            float targetDist = optimizedSegments[solvePathIndex];
            float currentDist = odometry.getSegmentMM();

            if (currentDist >= targetDist) {
                // Time to execute the next turn
                char turn = optimizedPath[solvePathIndex];

                if (client && client.connected()) {
                    client.printf("Seg %d/%d: %c @ %.0f/%.0fmm\n",
                        solvePathIndex + 1, optimizedPathLength,
                        turn, currentDist, targetDist);
                }

                executeTurn(turn);
                odometry.resetSegment();
                solvePathIndex++;
            }
        }
        break;
    }

    // ----------------------------------------------------------------
    case FINISHED:
    {
        // Victory blink
        for (int i = 0; i < 4; i++) {
            digitalWrite(ONBOARD_LED, HIGH);
            delay(30);
            digitalWrite(ONBOARD_LED, LOW);
            delay(20);
        }
        break;
    }

    case CALIBRATING:
        break;
    }
}

// =====================================================================
//  WIFI SETUP
// =====================================================================
void setupWiFi() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║              WiFi Setup                ║");
    Serial.println("╚════════════════════════════════════════╝");

    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    delay(100);

    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        if (attempts % 10 == 9) {
            Serial.print(" [Status: ");
            switch (WiFi.status()) {
                case WL_IDLE_STATUS:    Serial.print("IDLE"); break;
                case WL_NO_SSID_AVAIL: Serial.print("NO SSID"); break;
                case WL_CONNECT_FAILED: Serial.print("FAILED"); break;
                case WL_DISCONNECTED:   Serial.print("DISC"); break;
                default: Serial.print(WiFi.status());
            }
            Serial.println("]");
        }
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi Connected!");
        Serial.print("✓ IP: ");
        Serial.println(WiFi.localIP());
        Serial.printf("✓ RSSI: %d dBm\n", WiFi.RSSI());
        server.begin();
        Serial.println("✓ Telnet Server Started on port 23");
    } else {
        Serial.println("\n❌ WiFi Connection Failed!");
        Serial.println("Scanning networks...");
        int n = WiFi.scanNetworks();
        if (n == 0) Serial.println("  No networks found!");
        else {
            Serial.printf("  Found %d networks:\n", n);
            for (int i = 0; i < n && i < 10; i++)
                Serial.printf("    %d: %s (%d dBm)\n", i+1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
    }
}

void handleWiFiClient() {
    if (server.hasClient()) {
        if (!client || !client.connected()) {
            if (client) client.stop();
            client = server.available();
            if (client) {
                Serial.println("✓ Telnet client connected");
                client.println("╔════════════════════════════════════════╗");
                client.println("║  Cascade PID Maze Solver Console      ║");
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

// =====================================================================
//  COMMAND PROCESSING
// =====================================================================
void processCommand(String cmd) {
    cmd.toUpperCase();

    // === BASIC COMMANDS ===
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
        resetControlState();
        junctionCount = 0;
        pathIndex = 0;
        rawPath = "";
        optimizedPath = "";
        client.println("✓ RESET — Ready for Run 1");
    }
    else if (cmd == "STATUS" || cmd == "ST") {
        printStatus();
    }
    else if (cmd == "PATH") {
        client.println("\n=== Path Info ===");
        client.printf("Raw: %s (%d moves)\n", rawPath.c_str(), rawPath.length());
        client.printf("Optimized: %s (%d moves)\n", optimizedPath.c_str(), optimizedPath.length());
        if (rawPath.length() > 0 && optimizedPath.length() > 0) {
            client.printf("Saved: %d moves\n", rawPath.length() - optimizedPath.length());
        }
        client.println("Segment distances (mm):");
        int maxSeg = (currentState == SOLVING || currentState == FINISHED) ? optimizedPathLength : pathIndex;
        float* segs = (currentState == SOLVING || currentState == FINISHED) ? optimizedSegments : pathSegments;
        for (int i = 0; i < maxSeg; i++) {
            client.printf("  [%d] %.0f mm\n", i, segs[i]);
        }
        client.println("=================\n");
    }
    else if (cmd == "HELP" || cmd == "H") {
        printMenu();
    }

    // === STEERING PID TUNING ===
    else if (cmd.startsWith("SKP ")) {
        float v = cmd.substring(4).toFloat();
        steerPID.SetTunings(v, steerPID.GetKi(), steerPID.GetKd());
        client.printf("✓ Steer Kp = %.5f\n", v);
    }
    else if (cmd.startsWith("SKI ")) {
        float v = cmd.substring(4).toFloat();
        steerPID.SetTunings(steerPID.GetKp(), v, steerPID.GetKd());
        client.printf("✓ Steer Ki = %.5f\n", v);
    }
    else if (cmd.startsWith("SKD ")) {
        float v = cmd.substring(4).toFloat();
        steerPID.SetTunings(steerPID.GetKp(), steerPID.GetKi(), v);
        client.printf("✓ Steer Kd = %.5f\n", v);
    }
    // Legacy KP/KI/KD commands → map to steering PID
    else if (cmd.startsWith("KP ")) {
        float v = cmd.substring(3).toFloat();
        steerPID.SetTunings(v, steerPID.GetKi(), steerPID.GetKd());
        client.printf("✓ Steer Kp = %.5f\n", v);
    }
    else if (cmd.startsWith("KI ")) {
        float v = cmd.substring(3).toFloat();
        steerPID.SetTunings(steerPID.GetKp(), v, steerPID.GetKd());
        client.printf("✓ Steer Ki = %.5f\n", v);
    }
    else if (cmd.startsWith("KD ")) {
        float v = cmd.substring(3).toFloat();
        steerPID.SetTunings(steerPID.GetKp(), steerPID.GetKi(), v);
        client.printf("✓ Steer Kd = %.5f\n", v);
    }

    // === VELOCITY PID TUNING ===
    else if (cmd.startsWith("VKP ")) {
        float v = cmd.substring(4).toFloat();
        motors.setVelPIDGains(v, VEL_KI_DEFAULT, VEL_KD_DEFAULT);
        client.printf("✓ Vel Kp = %.3f\n", v);
    }
    else if (cmd.startsWith("VKI ")) {
        float v = cmd.substring(4).toFloat();
        motors.setVelPIDGains(VEL_KP_DEFAULT, v, VEL_KD_DEFAULT);
        client.printf("✓ Vel Ki = %.3f\n", v);
    }
    else if (cmd.startsWith("VKD ")) {
        float v = cmd.substring(4).toFloat();
        motors.setVelPIDGains(VEL_KP_DEFAULT, VEL_KI_DEFAULT, v);
        client.printf("✓ Vel Kd = %.3f\n", v);
    }
    else if (cmd.startsWith("VTUNE ")) {
        // VTUNE <kp> <ki> <kd>
        int s1 = cmd.indexOf(' ', 6);
        int s2 = cmd.indexOf(' ', s1 + 1);
        if (s1 > 0 && s2 > 0) {
            float kp = cmd.substring(6, s1).toFloat();
            float ki = cmd.substring(s1+1, s2).toFloat();
            float kd = cmd.substring(s2+1).toFloat();
            motors.setVelPIDGains(kp, ki, kd);
            client.printf("✓ Vel PID: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
        }
    }
    else if (cmd.startsWith("FFGAIN ")) {
        float g = cmd.substring(7).toFloat();
        motors.setFeedforwardGain(g);
        client.printf("✓ Feedforward gain = %.3f (PWM per mm/s)\n", motors.getFeedforwardGain());
    }

    // === SPEED SETTINGS ===
    else if (cmd.startsWith("CRUISE ")) {
        currentCruiseSpeed = constrain(cmd.substring(7).toFloat(), MIN_SPEED_MMS, maxSpeedMMS);
        client.printf("✓ Cruise speed = %.0f mm/s\n", currentCruiseSpeed);
    }
    else if (cmd.startsWith("SPEED ")) {
        // Legacy: map raw PWM-like value to mm/s (rough conversion)
        float rawSpeed = cmd.substring(6).toFloat();
        currentCruiseSpeed = constrain(rawSpeed, MIN_SPEED_MMS, maxSpeedMMS);
        client.printf("✓ Cruise speed = %.0f mm/s\n", currentCruiseSpeed);
    }
    else if (cmd.startsWith("MAXSPEED ")) {
        maxSpeedMMS = constrain(cmd.substring(9).toFloat(), MIN_SPEED_MMS, 1000.0f);
        steerPID.SetOutputLimits(-maxSpeedMMS, maxSpeedMMS);
        client.printf("✓ Max speed = %.0f mm/s\n", maxSpeedMMS);
    }
    else if (cmd.startsWith("TURNSPEED ")) {
        turnSpeedMMS = constrain(cmd.substring(10).toFloat(), MIN_SPEED_MMS, maxSpeedMMS);
        client.printf("✓ Turn speed = %.0f mm/s\n", turnSpeedMMS);
    }

    // === JUNCTION TUNING ===
    else if (cmd.startsWith("JCONFIRM ")) {
        junctionConfirmMM = constrain(cmd.substring(9).toFloat(), 3.0f, 100.0f);
        client.printf("✓ Junction confirm distance = %.1f mm\n", junctionConfirmMM);
    }
    else if (cmd.startsWith("JSPACE ")) {
        junctionMinSpacing = constrain(cmd.substring(7).toFloat(), 10.0f, 500.0f);
        client.printf("✓ Junction min spacing = %.1f mm\n", junctionMinSpacing);
    }
    else if (cmd.startsWith("HTOL ")) {
        headingTolerance = constrain(cmd.substring(5).toFloat(), 0.01f, 0.5f);
        client.printf("✓ Heading tolerance = %.3f rad (%.1f°)\n",
            headingTolerance, headingTolerance * 180.0f / M_PI);
    }

    // === STATE CHANGE ===
    else if (cmd == "WAIT") {
        currentState = WAIT_FOR_RUN_2;
        client.println("✓ Switched to WAIT_FOR_RUN_2");
    }

    // === TESTING ===
    else if (cmd == "TEST" || cmd == "T") {
        bool sv[8];
        sensors.getSensorArray(sv);
        client.println("\n=== Sensor Test ===");
        client.print("Pattern: [");
        for (int i = 0; i < 8; i++) client.print(sv[i] ? "█" : "·");
        client.println("]");
        client.printf("Error: %.2f\n", sensors.getLineError());
        client.printf("Active: %d sensors\n", sensors.getActiveSensorCount());
        client.printf("Mask: 0x%02X\n", sensors.getActiveSensorMask());
        client.printf("Left branch: %s\n", sensors.hasLeftBranch() ? "YES" : "NO");
        client.printf("Right branch: %s\n", sensors.hasRightBranch() ? "YES" : "NO");
        client.printf("Straight: %s\n", sensors.hasStraight() ? "YES" : "NO");
        client.printf("Line end: %s\n", sensors.isLineEnd() ? "YES" : "NO");
        client.printf("Endpoint: %s\n", sensors.isEndPoint() ? "YES" : "NO");
        client.println("==================\n");
    }
    else if (cmd == "ENCTEST") {
        motors.clearEncoders();
        motors.setSpeeds(100, 100);
        delay(1000);
        motors.stopBrake();
        long left = motors.getLeftCount();
        long right = motors.getRightCount();
        client.printf("Left:  %ld  (%.1f mm)\n", left, left * MM_PER_TICK);
        client.printf("Right: %ld  (%.1f mm)\n", right, right * MM_PER_TICK);
        client.printf("Diff:  %ld  (%.1f%%)\n",
            abs(left - right),
            100.0 * abs(left - right) / max(left, right));
    }
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
        for (int i = 0; i < 8; i++) client.print(digital[i] ? "█" : "·");
        client.println("]");
        for (int i = 0; i < 8; i++) {
            client.printf("  S%-2d | %-6d | %s\n", i+1, rawValues[i], digital[i] ? "LINE" : "-");
        }
        client.printf("Sensitivity: %.2f\n", sensors.getSensitivity());
        client.println("===========================\n");
    }
    else if (cmd.startsWith("SENS ")) {
        float sens = cmd.substring(5).toFloat();
        sensors.setSensitivity(sens);
        client.printf("✓ Sensitivity = %.2f\n", sensors.getSensitivity());
        sensors.printCalibrationToClient(client);
    }
    else if (cmd == "PID") {
        client.println("\n=== PID Values ===");
        client.printf("Steer: Kp=%.5f Ki=%.5f Kd=%.5f\n",
            steerPID.GetKp(), steerPID.GetKi(), steerPID.GetKd());
        client.printf("Steer Output: %.2f  Input: %.2f  Setpoint: %.2f\n",
            steerOutput, steerInput, steerSetpoint);
        client.printf("Vel L Target: %.1f mm/s  R Target: %.1f mm/s\n",
            motors.getLeftTargetMMS(), motors.getRightTargetMMS());
        client.printf("Vel L PWM: %.0f   R PWM: %.0f\n",
            motors.getLeftPWMOut(), motors.getRightPWMOut());
        client.println("==================\n");
    }
    else if (cmd == "ODO") {
        client.println("\n=== Odometry ===");
        client.printf("Pos: (%.1f, %.1f) mm\n", odometry.getX(), odometry.getY());
        client.printf("Heading: %.2f rad (%.1f°)\n",
            odometry.getHeading(), odometry.getHeading() * 180.0f / M_PI);
        client.printf("Segment: %.1f mm\n", odometry.getSegmentMM());
        client.printf("Velocity: %.1f mm/s  ω: %.2f rad/s\n",
            odometry.getLinearVelocity(), odometry.getAngularVelocity());
        client.println("================\n");
    }
    else if (cmd == "DIMS") {
        client.println("\n=== Robot Dimensions ===");
        client.printf("Wheel Ø:    %.1f mm\n", WHEEL_DIAMETER_MM);
        client.printf("Wheel base: %.1f mm\n", WHEEL_BASE_MM);
        client.printf("Encoder CPR: %.0f\n", ENCODER_CPR);
        client.printf("mm/tick:     %.4f\n", MM_PER_TICK);
        client.printf("ticks/mm:    %.3f\n", TICKS_PER_MM);
        client.printf("Sensor offset: %.1f mm\n", SENSOR_OFFSET_MM);
        client.printf("Castor offset: %.1f mm\n", CASTOR_OFFSET_MM);
        client.printf("Sensor pitch: %.1f mm\n", SENSOR_PITCH_MM);
        client.printf("Line width:   %.1f mm\n", LINE_WIDTH_MM);
        client.println("========================\n");
    }
    else {
        client.println("❌ Unknown. Type HELP");
    }
}

// =====================================================================
//  MENU & STATUS
// =====================================================================
void printMenu() {
    client.println("\n=== Commands ===");
    client.println("START/GO     - Start robot");
    client.println("STOP/S       - Stop robot");
    client.println("RESET/R      - Reset to Run 1");
    client.println("HELP/H       - This menu");
    client.println("STATUS/ST    - Show status");
    client.println("PATH         - Show path info");
    client.println("");
    client.println("=== Steering PID ===");
    client.println("SKP/SKI/SKD <val> - Steering PID gains");
    client.println("KP/KI/KD <val>    - (legacy, same as SKP/SKI/SKD)");
    client.println("");
    client.println("=== Velocity PID ===");
    client.println("VKP/VKI/VKD <val>      - Velocity PID gains");
    client.println("VTUNE <kp> <ki> <kd>   - Set all velocity PID");
    client.println("");
    client.println("=== Speeds (mm/s) ===");
    client.println("CRUISE <val>     - Cruise speed");
    client.println("MAXSPEED <val>   - Max speed");
    client.println("TURNSPEED <val>  - Turn speed");
    client.println("");
    client.println("=== Junction ===");
    client.println("JCONFIRM <mm>  - Junction confirm distance");
    client.println("JSPACE <mm>    - Min junction spacing");
    client.println("HTOL <rad>     - Heading tolerance");
    client.println("");
    client.println("=== Sensors ===");
    client.println("TEST/T  - Sensor test");
    client.println("RAW     - Raw sensor values");
    client.println("CAL     - Calibration data");
    client.println("SENS <0-1> - Sensitivity");
    client.println("");
    client.println("=== Info ===");
    client.println("PID   - PID state");
    client.println("ODO   - Odometry state");
    client.println("DIMS  - Robot dimensions");
    client.println("ENCTEST - Encoder test");
    client.println("================\n");
}

void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    switch (currentState) {
        case WAIT_FOR_RUN_1: client.println("WAIT_RUN_1"); break;
        case MAPPING:        client.println("MAPPING"); break;
        case OPTIMIZING:     client.println("OPTIMIZING"); break;
        case WAIT_FOR_RUN_2: client.println("WAIT_RUN_2"); break;
        case SOLVING:        client.println("SOLVING"); break;
        case FINISHED:       client.println("FINISHED"); break;
        default:             client.println("CALIBRATING");
    }
    client.printf("Turn mode: %d\n", turnMode);
    client.printf("Steer PID: Kp=%.5f Ki=%.5f Kd=%.5f\n",
        steerPID.GetKp(), steerPID.GetKi(), steerPID.GetKd());
    client.printf("Cruise: %.0f mm/s  Max: %.0f mm/s  Turn: %.0f mm/s\n",
        currentCruiseSpeed, maxSpeedMMS, turnSpeedMMS);
    client.printf("Junction confirm: %.1f mm  Spacing: %.1f mm\n",
        junctionConfirmMM, junctionMinSpacing);
    client.printf("Heading: %.2f rad  Target: %.2f rad\n",
        odometry.getHeading(), targetHeading);
    client.printf("Segment: %.1f mm\n", odometry.getSegmentMM());
    client.printf("Junctions: %d  PathIdx: %d\n", junctionCount, pathIndex);
    client.printf("Error: %.2f\n", sensors.getLineError());
    client.printf("On Line: %s\n", sensors.onLine() ? "YES" : "NO");
    client.println("==============\n");
}