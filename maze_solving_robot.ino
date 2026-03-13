#include <Arduino.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <cstdint>

// === HARDWARE PINS ===
#define SENSOR_PIN_1  32  
#define SENSOR_PIN_2  33
#define SENSOR_PIN_3  25
#define SENSOR_PIN_4  26
#define SENSOR_PIN_5  27
#define SENSOR_PIN_6  14
#define SENSOR_PIN_7  12
#define SENSOR_PIN_8  13  

#define ENCODER_L_A 19      
#define ENCODER_L_B 21      
#define ENCODER_R_A 39      
#define ENCODER_R_B 36      

#define MOTOR_L_AIN1 17
#define MOTOR_L_AIN2 16   
#define MOTOR_R_BIN1 18   
#define MOTOR_R_BIN2 5
#define MOTOR_L_PWMA 4
#define MOTOR_R_PWMB 22     
// Note: STBY is hardwired to VCC, no pin needed.

#define ONBOARD_LED  2
#define BOOT_BUTTON  0      // Physical BOOT button on ESP32

// === WIFI CREDENTIALS (UPDATE THESE!) ===
#define WIFI_SSID "KKS's phone"
#define WIFI_PASS "kvsandkks"
#define TELNET_PORT 23

// === KINEMATIC CONSTANTS (Calculated: 400 CPR, 118mm W, 44mm D) ===
const int TICKS_NODE_ALIGN = 344; // 119mm offset to center wheels
const int TICKS_90_DEG = 268;     // 90 degree pivot
const int TICKS_180_DEG = 536;    // 180 degree pivot
const int TICKS_CLEAR_NODE = 100; // Small push to clear junction after a 'Straight'
const int LINE_THRESHOLD = 700;   // >700 means WHITE line detected

// --- OBJECTS & SERVERS ---
const uint8_t SensorCount = 8;
const uint8_t SensorPins[] = {SENSOR_PIN_1, SENSOR_PIN_2, SENSOR_PIN_3, SENSOR_PIN_4, SENSOR_PIN_5, SENSOR_PIN_6, SENSOR_PIN_7, SENSOR_PIN_8};
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

// --- VOLATILE ENCODER COUNTS ---
volatile int32_t encLeftCount = 0;
volatile int32_t encRightCount = 0;

// --- PID CONSTANTS & TUNING ---
float Kp_out = 0.05, Kd_out = 0.5;
float Kp_in = 1.2, Ki_in = 0.0, Kd_in = 0.1;
int baseSpeed = 100; // Line following speed
int turnSpeed = 80;  // Kinematic pivot speed
int maxPWM = 255;

// --- STATE VARIABLES ---
int lastErrorLine = 0, integralLeft = 0, integralRight = 0;
int lastErrorLeft = 0, lastErrorRight = 0;

// --- MAZE MEMORY & STATE MACHINE ---
enum RobotMode { EXPLORING, FINISHED, FAST_RUN };
RobotMode currentMode = EXPLORING;
char path[250];
int pathLength = 0;
int pathIndex = 0;

// ==========================================
// ULTRA-FAST ISRs (Direct Register Reads)
// ==========================================
void IRAM_ATTR isrLeftA() {
  uint32_t gpio_status = REG_READ(GPIO_IN_REG);
  bool stateA = (gpio_status >> ENCODER_L_A) & 1;
  bool stateB = (gpio_status >> ENCODER_L_B) & 1;
  if (stateA == stateB) encLeftCount++; else encLeftCount--;
}

void IRAM_ATTR isrRightA() {
  uint32_t gpio_status1 = REG_READ(GPIO_IN1_REG);
  bool stateA = (gpio_status1 >> (ENCODER_R_A - 32)) & 1;
  bool stateB = (gpio_status1 >> (ENCODER_R_B - 32)) & 1;
  if (stateA == stateB) encRightCount++; else encRightCount--;
}

// ==========================================
// MOTOR DRIVER HELPER (FIXED FOR CORE V3)
// ==========================================
void setMotors(int pwmLeft, int pwmRight) {
  pwmLeft = constrain(pwmLeft, -maxPWM, maxPWM);
  pwmRight = constrain(pwmRight, -maxPWM, maxPWM);

  if (pwmLeft >= 0) {
    digitalWrite(MOTOR_L_AIN1, HIGH); digitalWrite(MOTOR_L_AIN2, LOW);
  } else {
    digitalWrite(MOTOR_L_AIN1, LOW); digitalWrite(MOTOR_L_AIN2, HIGH); pwmLeft = -pwmLeft;
  }
  
  if (pwmRight >= 0) {
    digitalWrite(MOTOR_R_BIN1, HIGH); digitalWrite(MOTOR_R_BIN2, LOW);
  } else {
    digitalWrite(MOTOR_R_BIN1, LOW); digitalWrite(MOTOR_R_BIN2, HIGH); pwmRight = -pwmRight;
  }
  
  // FIXED: Core v3 uses the actual PIN, not the channel
  ledcWrite(MOTOR_L_PWMA, pwmLeft); 
  ledcWrite(MOTOR_R_PWMB, pwmRight);
}

// ==========================================
// KINEMATIC MOVEMENT FUNCTIONS (Blocking)
// ==========================================
void resetEncoders() {
  noInterrupts(); encLeftCount = 0; encRightCount = 0; interrupts();
}

void driveTicks(int targetTicks, int speedLeft, int speedRight) {
  resetEncoders();
  setMotors(speedLeft, speedRight);
  while(true) {
    noInterrupts();
    int32_t currentL = abs(encLeftCount);
    int32_t currentR = abs(encRightCount);
    interrupts();
    if (currentL >= targetTicks || currentR >= targetTicks) break;
    vTaskDelay(1); // Feed FreeRTOS watchdog
  }
  setMotors(0, 0); // Hard stop
  delay(100);      // Settle chassis
}

void alignNode() { driveTicks(TICKS_NODE_ALIGN, baseSpeed, baseSpeed); }
void turnLeft90() { driveTicks(TICKS_90_DEG, -turnSpeed, turnSpeed); }
void turnRight90() { driveTicks(TICKS_90_DEG, turnSpeed, -turnSpeed); }
void uTurn() { driveTicks(TICKS_180_DEG, turnSpeed, -turnSpeed); }
void clearNode() { driveTicks(TICKS_CLEAR_NODE, baseSpeed, baseSpeed); }

// ==========================================
// PATH OPTIMIZATION (Tremaux's Algorithm)
// ==========================================
void simplifyPath() {
  if (pathLength < 3 || path[pathLength - 2] != 'B') return;
  
  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    switch (path[pathLength - i]) {
      case 'R': totalAngle += 90; break;
      case 'L': totalAngle += 270; break;
      case 'B': totalAngle += 180; break;
    }
  }
  totalAngle = totalAngle % 360;
  switch (totalAngle) {
    case 0: path[pathLength - 3] = 'S'; break;
    case 90: path[pathLength - 3] = 'R'; break;
    case 180: path[pathLength - 3] = 'B'; break;
    case 270: path[pathLength - 3] = 'L'; break;
  }
  pathLength -= 2;
}

// ==========================================
// CORE 1: MAZE SOLVING & REAL-TIME CONTROL
// ==========================================
void ControlLoopTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz Loop

  while (true) {
    
    // --- STATE 1: WAITING AT FINISH LINE ---
    if (currentMode == FINISHED) {
      digitalWrite(ONBOARD_LED, HIGH); 
      setMotors(0, 0); 
      
      // Wait for physical BOOT button press to start the Fast Run
      if (digitalRead(BOOT_BUTTON) == LOW) {
        vTaskDelay(1000); // Debounce and allow hand removal
        currentMode = FAST_RUN;
        pathIndex = 0;    // Reset read index to start of memory
        digitalWrite(ONBOARD_LED, LOW);
        if(telnetClient) telnetClient.println("\n*** INITIATING FAST RUN ***");
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue; 
    }

    // --- SENSOR READING ---
    int position = qtr.readLineWhite(sensorValues);
    
    bool leftJunction = sensorValues[0] > LINE_THRESHOLD;
    bool rightJunction = sensorValues[7] > LINE_THRESHOLD;
    bool centerOnLine = (sensorValues[3] > LINE_THRESHOLD) || (sensorValues[4] > LINE_THRESHOLD);
    bool endOfMaze = leftJunction && rightJunction && centerOnLine && (sensorValues[1] > LINE_THRESHOLD);


    // --- STATE 2: EXPLORATION MODE (MAPPING) ---
    if (currentMode == EXPLORING) {
      if (endOfMaze) {
        alignNode();
        currentMode = FINISHED; 
        if(telnetClient) telnetClient.println("\nMAZE MAPPED! Waiting for BOOT button press...");
      } 
      else if (leftJunction) {
        alignNode(); turnLeft90();
        path[pathLength++] = 'L'; simplifyPath();
      } 
      else if (rightJunction && !centerOnLine) { // Pure Right
        alignNode(); turnRight90();
        path[pathLength++] = 'R'; simplifyPath();
      }
      else if (rightJunction && centerOnLine) { // Straight or Right
        alignNode();
        path[pathLength++] = 'S'; simplifyPath(); // Left-Hand Rule prioritizes straight
      }
      
      // Dead End Detection (Lost Line)
      if (sensorValues[0] < 200 && sensorValues[7] < 200 && !centerOnLine) {
        alignNode(); // Move slightly forward to confirm
        qtr.readLineWhite(sensorValues);
        if((sensorValues[3] < 200) && (sensorValues[4] < 200)) {
          uTurn();
          path[pathLength++] = 'B'; simplifyPath();
        }
        continue;
      }
    } 
    
    // --- STATE 3: FAST RUN MODE (SOLVING) ---
    else if (currentMode == FAST_RUN) {
      if (endOfMaze) {
        alignNode();
        currentMode = FINISHED; 
        if(telnetClient) telnetClient.println("\nFAST RUN COMPLETE!");
      }
      else if (leftJunction || rightJunction) {
        alignNode(); // Center the wheels on the junction
        
        if (pathIndex < pathLength) {
          char nextTurn = path[pathIndex++]; 
          
          if (nextTurn == 'L') {
            turnLeft90();
          } else if (nextTurn == 'R') {
            turnRight90();
          } else if (nextTurn == 'S') {
            clearNode(); // Drive straight slightly to clear junction sensors
          }
        }
      }
    }

    // --- NORMAL LINE FOLLOWING PID ---
    // (Executes when driving straight in both Exploring and Fast Run modes)
    int errorLine = 3500 - position;
    float deltaV = (Kp_out * errorLine) + (Kd_out * (errorLine - lastErrorLine));
    lastErrorLine = errorLine;

    int targetVelLeft = baseSpeed - deltaV;
    int targetVelRight = baseSpeed + deltaV;

    noInterrupts();
    int currentVelLeft = encLeftCount; int currentVelRight = encRightCount;
    encLeftCount = 0; encRightCount = 0;
    interrupts();

    int errorLeft = targetVelLeft - currentVelLeft;
    int errorRight = targetVelRight - currentVelRight;
    integralLeft += errorLeft; integralRight += errorRight;

    int pwmLeft = (Kp_in * errorLeft) + (Ki_in * integralLeft) + (Kd_in * (errorLeft - lastErrorLeft));
    int pwmRight = (Kp_in * errorRight) + (Ki_in * integralRight) + (Kd_in * (errorRight - lastErrorRight));

    lastErrorLeft = errorLeft; lastErrorRight = errorRight;

    setMotors(pwmLeft, pwmRight);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==========================================
// SETUP & CORE 0: WIFI / TELNET
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000); // Give Serial monitor a second to catch up

  qtr.setTypeRC();
  qtr.setSensorPins(SensorPins, SensorCount);

  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  pinMode(MOTOR_L_AIN1, OUTPUT); pinMode(MOTOR_L_AIN2, OUTPUT);
  pinMode(MOTOR_R_BIN1, OUTPUT); pinMode(MOTOR_R_BIN2, OUTPUT);

  // Core v3 API setup
  ledcAttach(MOTOR_L_PWMA, 20000, 8); 
  ledcAttach(MOTOR_R_PWMB, 20000, 8);

  pinMode(ENCODER_L_A, INPUT_PULLUP); pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT); pinMode(ENCODER_R_B, INPUT); // Requires physical pullups on 36/39!
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), isrRightA, CHANGE);

  Serial.println("\n--- Starting Calibration ---");
  Serial.println("Calibrating QTR for WHITE line...");
  setMotors(60, -60);
  for (uint16_t i = 0; i < 100; i++) { qtr.calibrate(); delay(10); }
  setMotors(-60, 60);
  for (uint16_t i = 0; i < 100; i++) { qtr.calibrate(); delay(10); }
  setMotors(0, 0);
  Serial.println("Calibration Done.");
  
  Serial.print("\nConnecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA); 
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n\n--- NETWORK SUCCESS ---");
  Serial.println("WiFi Connected!");
  Serial.print("MATLAB Telnet IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("-----------------------\n");
  
  telnetServer.begin(); 
  telnetServer.setNoDelay(true);

  xTaskCreatePinnedToCore(ControlLoopTask, "PID_Loop", 8192, NULL, 1, NULL, 1);
}

void loop() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
    } else telnetServer.available().stop();
  }

  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    String req = telnetClient.readStringUntil('\n');
    req.trim(); req.toUpperCase();
    
    if (req.startsWith("PO")) Kp_out = req.substring(2).toFloat();
    else if (req.startsWith("DO")) Kd_out = req.substring(2).toFloat();
    else if (req.startsWith("PI")) Kp_in = req.substring(2).toFloat();
    else if (req.startsWith("II")) Ki_in = req.substring(2).toFloat();
    else if (req.startsWith("DI")) Kd_in = req.substring(2).toFloat();
    else if (req.startsWith("BS")) baseSpeed = req.substring(2).toInt();
    
    else if (req.startsWith("PATH")) {
      telnetClient.print("Optimized Path Memory: ");
      for(int i=0; i<pathLength; i++) telnetClient.print(path[i]);
      telnetClient.println();
    }
  }
  delay(10);
}