#include <QTRSensors.h>
#include <EEPROM.h>

// Sensor configuration
#define SENSOR_CNT 5
#define ADDR_BASE 10
#define CENTER_VAL 2000
#define OUT_OF_LINE_ERROR_VALUE 20  // Changed from 3000 to match glider

// Motor pins
const int PWM1 = 6;
const int dir1 = 2;
const int PWM2 = 5;
const int dir2 = 3;

// Track type (from glider's Sensor.h)
#define BLACK_LINE_WHITE_TRACK 1
#define WHITE_LINE_BLACK_TRACK 0

// Speeds
const int BaseSpeed = 70;
const int MaxSpeed = 250;

// PID Configuration
#define DEFAULT_KP 0.05
#define DEFAULT_KD 0.4
#define DEFAULT_KI 0.0

// Timing (matching glider's defaults)
#define STOP_CHECK_DELAY 150  // Changed from 200 to match glider
#define BRAKE_DURATION_MILLIS 50
#define STOP_BRAKE_DURATION 1000  // For stop patch

// Turn speed reduction (from glider)
#define TURN_SPEED_REDUCTION_ENABLED 1
#define TURN_SPEED_REDUCTION_PERCENT 10

// QTR Sensor variables
QTRSensors qtr;
const uint8_t SensorCount = SENSOR_CNT;
unsigned int sensorValues[SensorCount];
int sensAvg[SENSOR_CNT];
bool isDebug = true;

// Track type (from glider)
uint8_t trackType = BLACK_LINE_WHITE_TRACK;

// PID Variables
int error = 0;
int error_dir = 0;
int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int PID_value = 0;
float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;

// Motor offset correction (from glider's MotorControl)
// NOTE: In glider, offsets are SUBTRACTED to slow down faster motors
int leftMotorOffset = 0;
int rightMotorOffset = 0;

// Loop delay (from glider) - allows tuning derivative response
int loopDelay = 0;

// Button variables
int button1 = 1;
int button2 = 1;
bool isCalibrated = false;
bool isRunning = false;

// Macros for sensor pattern checking (adapted from glider)
#define MID_3_SENSORS_HIGH (s1 == 1 && s2 == 1 && s3 == 1)
#define MID_3_SENSORS_LOW (s1 == 0 && s2 == 0 && s3 == 0)

// Function prototypes
void init_motor();
void motordrive(const char *directionL, int powerL, const char *directionR, int powerR);
void motorsteer(const char *robotdirection, int power);
void saveValue(int addr, uint16_t value);
uint16_t readValue(int addr);
void restoreSensorValue();
void storeSensorValue();
void initialize(bool debug);
void readButton();
int getButtonEvt(int idx);
void performCalibration();
void readSensors();
void calculatePID();
void controlMotors();
void shortBrake(int duration);
void turnCW(int leftSpeed, int rightSpeed);
void turnCCW(int leftSpeed, int rightSpeed);
void moveStraight(int leftSpeed, int rightSpeed);
void stopMotors();
bool isOutOfLine(unsigned int* sensors);
uint16_t getSensorReadings();
int getCalculatedError(int mode);
void indicateOn();
void indicateOff();
void indicateInversionOn();
void indicateInversionOff();

//=============================================================================
// LED/BUZZER INDICATORS (from glider)
//=============================================================================

/**
 * @brief Indicate checkpoint/junction detection (from glider's indicateOn)
 */
void indicateOn() {
  // You can add LED/buzzer logic here if you have them
  // For now, just print to serial
  static unsigned long lastIndicate = 0;
  if (millis() - lastIndicate > 500) {  // Debounce
    Serial.println(">>> CHECKPOINT/JUNCTION DETECTED <<<");
    lastIndicate = millis();
  }
}

/**
 * @brief Clear checkpoint indication
 */
void indicateOff() {
  // Clear LED/buzzer if you have them
}

/**
 * @brief Indicate track inversion (from glider's indicateInversionOn)
 */
void indicateInversionOn() {
  static bool lastState = false;
  if (!lastState) {
    Serial.println(">>> TRACK INVERTED: WHITE LINE ON BLACK <<<");
    lastState = true;
  }
}

/**
 * @brief Clear inversion indication
 */
void indicateInversionOff() {
  static bool lastState = true;
  if (lastState) {
    lastState = false;
  }
}

//=============================================================================
// MOTOR CONTROL FUNCTIONS (Fixed to match glider's MotorControl logic)
//=============================================================================

void init_motor() {
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void motordrive(const char *directionL, int powerL, const char *directionR, int powerR) {
  // NOTE: Do NOT apply offsets here - they're applied in controlMotors()
  
  if (strcmp(directionL, "forward") == 0) {
    digitalWrite(dir1, HIGH);
  } else if (strcmp(directionL, "backward") == 0) {
    digitalWrite(dir1, LOW);
  }

  if (strcmp(directionR, "forward") == 0) {
    digitalWrite(dir2, LOW);
  } else if (strcmp(directionR, "backward") == 0) {
    digitalWrite(dir2, HIGH);
  }

  analogWrite(PWM1, powerL);
  analogWrite(PWM2, powerR);
}

void motorsteer(const char *robotdirection, int power) {
  if (strcmp(robotdirection, "forward") == 0) {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power);
  } else if (strcmp(robotdirection, "stop") == 0) {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM2, 0);
  }
}

void moveStraight(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("forward", leftSpeed, "forward", rightSpeed);
}

void turnCW(int leftSpeed, int rightSpeed) {
#if TURN_SPEED_REDUCTION_ENABLED == 1
  leftSpeed = (leftSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightSpeed = (rightSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("forward", leftSpeed, "backward", rightSpeed);
}

void turnCCW(int leftSpeed, int rightSpeed) {
#if TURN_SPEED_REDUCTION_ENABLED == 1
  leftSpeed = (leftSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightSpeed = (rightSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("backward", leftSpeed, "forward", rightSpeed);
}

void shortBrake(int duration) {
  // digitalWrite(dir1, LOW);
  // analogWrite(PWM1, 255);
  // digitalWrite(dir2, HIGH);
  // analogWrite(PWM2, 255);
  // delay(duration);
  // stopMotors();
}

void stopMotors() {
  motorsteer("stop", 0);
}

//=============================================================================
// BUTTON FUNCTIONS
//=============================================================================

void readButton() {
  int adcValue = analogRead(A7);
  if (adcValue < 100) {
    button1 = 0;
    button2 = 1;
  } else if (adcValue < 500) {
    button1 = 1;
    button2 = 0;
  } else {
    button1 = 1;
    button2 = 1;
  }
}

int getButtonEvt(int idx) {
  return idx == 1 ? button1 : button2;
}

void performCalibration() {
  Serial.println("========================================");
  Serial.println("CALIBRATION MODE");
  Serial.println("Move robot left and right over the line");
  Serial.println("Calibration will run for 3 seconds...");
  Serial.println("========================================");
  
  delay(1000);
  qtr.resetCalibration();
  
  unsigned long calibrationStart = millis();
  while (millis() - calibrationStart < 3000) {
    qtr.calibrate();
    delay(10);
  }
  
  storeSensorValue();
  
  Serial.println("========================================");
  Serial.println("CALIBRATION COMPLETE!");
  Serial.println("Sensor values:");
  for (int i = 0; i < SENSOR_CNT; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(", Max=");
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(", Avg=");
    Serial.println(sensAvg[i]);
  }
  Serial.print("Track Type: ");
  Serial.println(trackType == BLACK_LINE_WHITE_TRACK ? "BLACK_LINE_WHITE_TRACK" : "WHITE_LINE_BLACK_TRACK");
  Serial.println("========================================");
  
  isCalibrated = true;
}

//=============================================================================
// EEPROM FUNCTIONS
//=============================================================================

void saveValue(int addr, uint16_t value) {
  EEPROM.write(addr, value >> 8);
  EEPROM.write(addr + 1, value & 0xFF);
}

uint16_t readValue(int addr) {
  return (EEPROM.read(addr) << 8) + EEPROM.read(addr + 1);
}

void restoreSensorValue() {
  qtr.calibrate();
  for (int i = 0; i < SENSOR_CNT; i++) {
    int offset = i * 2;
    qtr.calibrationOn.minimum[i] = readValue(ADDR_BASE + offset);
    qtr.calibrationOn.maximum[i] = readValue(ADDR_BASE * 5 + offset);
    sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;

    if (isDebug) {
      Serial.print(i);
      Serial.print(" - Min: ");
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(", Max: ");
      Serial.println(qtr.calibrationOn.maximum[i]);
    }
  }
}

void storeSensorValue() {
  for (int i = 0; i < SENSOR_CNT; i++) {
    int offset = i * 2;
    saveValue(ADDR_BASE + offset, qtr.calibrationOn.minimum[i]);
    saveValue(ADDR_BASE * 5 + offset, qtr.calibrationOn.maximum[i]);
    sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;
  }
}

void initialize(bool debug) {
  isDebug = debug;
  
  if (isDebug) {
    Serial.begin(9600);
  }
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A6, A3, A2, A1, A0 }, SensorCount);
  qtr.releaseEmitterPins();

  restoreSensorValue();
}

//=============================================================================
// SENSOR READING (Fixed to match glider's Sensor.cpp logic)
//=============================================================================

/**
 * @brief Get raw sensor readings as a bitmask (from glider's Sensor.cpp)
 */
uint16_t getSensorReadings() {
  qtr.readLineBlack(sensorValues);
  
  uint16_t sensorData = 0;
  
  for (int i = 0; i < SENSOR_CNT; i++) {
    if (sensorValues[i] > sensAvg[i]) {
      sensorData |= (1 << i);
    }
  }
  
  return sensorData;
}

/**
 * @brief Calculate error from sensor position (from glider's Sensor.cpp)
 */
int getCalculatedError(int mode) {
  uint16_t position;
  
  if (mode == 0 || trackType == BLACK_LINE_WHITE_TRACK) {
    position = qtr.readLineBlack(sensorValues);
  } else {
    position = qtr.readLineWhite(sensorValues);
  }
  
  int calculatedError = position - CENTER_VAL;
  
  return calculatedError;
}

bool isOutOfLine(unsigned int* sensors) {
  for (int i = 0; i < SENSOR_CNT; i++) {
    if (sensors[i] > sensAvg[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Main sensor reading function (from glider's main.cpp lines 233-276)
 * FIXED: Now matches glider's order and logic exactly
 */
void readSensors() {
  uint16_t sensorData = getSensorReadings();
  
  // Calculate error first (glider line 234)
  error = getCalculatedError(0);
  
  // Extract individual sensor states
  int s0 = (sensorData & (1 << 0)) >> 0;  // Leftmost
  int s1 = (sensorData & (1 << 1)) >> 1;
  int s2 = (sensorData & (1 << 2)) >> 2;  // Center
  int s3 = (sensorData & (1 << 3)) >> 3;
  int s4 = (sensorData & (1 << 4)) >> 4;  // Rightmost
  
  // Track direction: which side saw the line last (glider line 237)
  if (s0 != s4) {
    error_dir = s0 - s4;
  }
  
  // CHECKPOINT/JUNCTION INDICATION (glider lines 239-242)
  // Uses middle 3 sensors (adapted from glider's MID_6_SENSORS_HIGH)
  if (MID_3_SENSORS_HIGH) {
    indicateOn();
  } else {
    indicateOff();
  }
  
  // TRACK INVERSION INDICATION (glider lines 244-247)
  if (trackType == WHITE_LINE_BLACK_TRACK) {
    indicateInversionOn();
  } else {
    indicateInversionOff();
  }
  
  // OUT OF LINE DETECTION - FIRST (glider lines 249-258)
  if (sensorData == 0b00000) {  // All sensors LOW = out of line
    if (error_dir < 0) {
      error = OUT_OF_LINE_ERROR_VALUE;  // Turn right
    } else if (error_dir > 0) {
      error = -1 * OUT_OF_LINE_ERROR_VALUE;  // Turn left (note: glider uses -1 * )
    }
  }
  // STOP PATCH DETECTION - SECOND (glider lines 260-271)
  else if (sensorData == 0b11111) {  // All sensors HIGH = stop patch
    // Move forward and verify it's really a stop patch
    moveStraight(BaseSpeed, BaseSpeed);
    delay(STOP_CHECK_DELAY);  // 150ms (matches glider)
    
    uint16_t sensorDataAgain = getSensorReadings();
    
    if (sensorDataAgain == 0b11111) {  // Still all HIGH?
      // indicateOff();  // Turn off indicators (glider line 267)
      // shortBrake(STOP_BRAKE_DURATION);  // 1 second brake (glider line 268)
      // stopMotors();  // Stop (glider line 269)
      
      Serial.println("STOP PATCH DETECTED - Stopping for 10 seconds");
      // delay(10000);  // 10 second delay (glider line 270)
    }
  }
  
  if (isDebug) {
    static int debugCounter = 0;
    if (++debugCounter % 10 == 0) {
      char sensorPattern[6];
      sprintf(sensorPattern, "%d%d%d%d%d", s0, s1, s2, s3, s4);
      
      Serial.print("Sensors: ");
      Serial.print(sensorPattern);
      Serial.print(" | Raw: ");
      for (int i = 0; i < SENSOR_CNT; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
      }
      Serial.print("| Error: ");
      Serial.print(error);
      Serial.print(" | Dir: ");
      Serial.print(error_dir);
      Serial.print(" | Track: ");
      Serial.println(trackType == BLACK_LINE_WHITE_TRACK ? "BL/WT" : "WL/BL");
    }
  }
}

//=============================================================================
// PID CALCULATION (from glider's main.cpp lines 281-294)
//=============================================================================

void calculatePID() {
  P = error;
  
  // Reset integral when on target (anti-windup)
  if (error == 0) {
    I = 0;
  } else {
    I = I + error;
  }
  
  // Constrain integral to prevent windup
  I = constrain(I, -200, 200);
  
  D = error - previousError;
  
  // Calculate PID correction value
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  PID_value = constrain(PID_value, -MaxSpeed, MaxSpeed);
  
  previousError = error;
}

//=============================================================================
// MOTOR CONTROL (FIXED to match glider's main.cpp lines 299-352)
//=============================================================================

void controlMotors() {

  // Special case: Out of line on the right (glider lines 299-312)
  if (error == OUT_OF_LINE_ERROR_VALUE) {
    Serial.println("OUT OF LINE - Turning Clockwise");

    // Apply brake to stop momentum
    // shortBrake(BRAKE_DURATION_MILLIS);
    
    // Keep turning clockwise until line is found
    uint16_t sensorReadings = getSensorReadings();
    while (isOutOfLine(sensorValues)) {
      // NOTE: Glider subtracts offsets from base speed (lines 306-307)
      turnCW(BaseSpeed - leftMotorOffset, BaseSpeed - rightMotorOffset);
      sensorReadings = getSensorReadings();
    }
    
    error_dir = 0;  // Reset direction tracking
  }
  // Special case: Out of line on the left (glider lines 315-328)
  else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE)) {  // Note: glider uses -1 *
    Serial.println("OUT OF LINE - Turning Counter-Clockwise");

    // Apply brake to stop momentum
    // shortBrake(BRAKE_DURATION_MILLIS);
    
    // Keep turning counter-clockwise until line is found
    uint16_t sensorReadings = getSensorReadings();
    while (isOutOfLine(sensorValues)) {
      // NOTE: Glider subtracts offsets from base speed (lines 322-323)
      turnCCW(BaseSpeed - leftMotorOffset, BaseSpeed - rightMotorOffset);
      sensorReadings = getSensorReadings();
    }
    
    error_dir = 0;  // Reset direction tracking
  }
  // Normal PID line following (glider lines 332-352)
  else {
    // Calculate individual motor speeds with PID correction
    // NOTE: Glider SUBTRACTS offsets to slow down faster motors (lines 345-346)
    int leftMotorSpeed = BaseSpeed + PID_value - leftMotorOffset;
    int rightMotorSpeed = BaseSpeed - PID_value - rightMotorOffset;
    
    // Move with calculated speeds
    moveStraight(leftMotorSpeed, rightMotorSpeed);
    
    // Use tunable loop delay (glider lines 349-350)
    if (D != 0) {
      delay(loopDelay);  // Changed from hardcoded 2 to variable loopDelay
    }
  }
}

//=============================================================================
// SETUP AND LOOP
//=============================================================================

void setup() {
  initialize(true);
  init_motor();
  
  Serial.println("========================================");
  Serial.println("LINE TRACER ROBOT - GLIDER LOGIC");
  Serial.println("========================================");
  Serial.println("Button 1: Calibrate sensors");
  Serial.println("Button 2: Start line tracing");
  Serial.println("========================================");
  Serial.println("Features:");
  Serial.println("- Motor offset compensation");
  Serial.println("- Track type inversion support");
  Serial.println("- Stop patch detection");
  Serial.println("- Checkpoint indication");
  Serial.println("- Out-of-line recovery");
  Serial.println("- PID line following");
  Serial.println("========================================");
  
  while (true) {
    readButton();
    
    if (getButtonEvt(1) == 0) {
      delay(200);
      while (getButtonEvt(1) == 0) {
        readButton();
        delay(10);
      }
      
      performCalibration();
      Serial.println("\nPress Button 2 to start line tracing");
      Serial.println("Press Button 1 to recalibrate");
    }
    
    if (getButtonEvt(2) == 0) {
      delay(200);
      while (getButtonEvt(2) == 0) {
        readButton();
        delay(10);
      }
      
      if (!isCalibrated) {
        Serial.println("ERROR: Please calibrate first (Button 1)");
        delay(2000);
      } else {
        Serial.println("========================================");
        Serial.println("STARTING LINE TRACING IN 3 SECONDS...");
        Serial.println("========================================");
        delay(3000);
        isRunning = true;
        break;
      }
    }
    
    delay(50);
  }
}

void loop() {
  if (isRunning) {
    // Three-step control loop (from glider's main.cpp lines 436-445)
    
    // Step 1: Read sensors and calculate error
    readSensors();
    
    // Step 2: Calculate PID correction
    calculatePID();
    
    // Step 3: Apply motor control
    controlMotors();
    
    // Check for stop button
    readButton();
    if (getButtonEvt(1) == 0 || getButtonEvt(2) == 0) {
      stopMotors();
      Serial.println("STOPPED - Reset to restart");
      while (true) { delay(1000); }
    }
  }
}