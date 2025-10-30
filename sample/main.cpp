#include <Arduino.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Motor pin mapping (TB6612)
#define AIN1 13
#define AIN2 12
#define PWMA 16
#define BIN1 18
#define BIN2 19
#define PWMB 17
#define STBY 23

// A is right
// B is left

// QTR-8A pins (ESP32 ADC pins)
const uint8_t NUM_SENSORS = 8;
const uint8_t QTR_PINS[NUM_SENSORS] = {26, 33, 32, 35, 34, 39, 36, 25};

// Button pins
#define BUTTON_CALIBRATE 2
#define BUTTON_START 4

bool lineTracingEnabled = false;

void waitForButton(int pin, const char *message)
{
  Serial.println(message);
  // Wait for button press (HIGH), debounce
  while (digitalRead(pin) == LOW)
    delay(10);
  delay(200); // Debounce
  while (digitalRead(pin) == HIGH)
    delay(10); // Wait for release
  delay(200);  // Debounce
}

// PID control constants
const float Kp = 0.08f;
const float Kd = 0.2f;
const int MaxSpeed = 150;
const int BaseSpeed = 120;
const int TurnSpeed = 80;
const int CENTER_POS = 2500;

// Junction detection settings
const int LINE_THRESHOLD = 500;
const int MIN_SENSORS_FOR_JUNCTION = 4;

// Junction action types
enum JunctionAction
{
  FORWARD,
  LEFT_90,
  RIGHT_90,
  LEFT_45,
  RIGHT_45,
  STOP,
  U_TURN,
  IGNORE
};

// Junction type enumeration
enum JunctionType
{
  ANY_JUNCTION,
  T_JUNCTION,
  LEFT_JUNCTION,
  RIGHT_JUNCTION,
  NO_JUNCTION
};

// Junction configuration structure
struct JunctionConfig
{
  int id;                    // Junction identifier (0, 1, 2, ...)
  JunctionAction action;     // What to do at this junction (FORWARD, LEFT_90, etc.)
  int preDelay;              // Pause time after detection before starting action (ms)
  int turnTime;              // Duration to execute the turn/action (ms)
  int postDelay;             // Wait time after action before resuming line following (ms)
  JunctionType junctionType; // Expected junction type
};

// Global variables
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];
int lastError = 0;

// Junction ignore period to prevent double triggers
bool junctionIgnoreActive = false;
unsigned long junctionIgnoreStartTime = 0;
const unsigned long JUNCTION_IGNORE_DURATION = 50; // ms

// Motor setup
const int MOTOR_A_OFFSET = 1;
const int MOTOR_B_OFFSET = -1;
Motor rightMotor = Motor(AIN1, AIN2, PWMA, MOTOR_A_OFFSET, STBY);
Motor leftMotor = Motor(BIN1, BIN2, PWMB, MOTOR_B_OFFSET, STBY);

// Function prototypes
void stopMotors();
void calibrateSensors();
void executeAction(JunctionAction action);
void performLineFollowing();
void turnUntilLine(JunctionAction action);

// Junction detection function
JunctionType detectJunctionType()
{
  // Count sensors above threshold
  int sensorsOnLine = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] > LINE_THRESHOLD)
      sensorsOnLine++;
  }

  // Require at least MIN_SENSORS_FOR_JUNCTION sensors to be triggered for a junction
  if (sensorsOnLine < MIN_SENSORS_FOR_JUNCTION)
    return NO_JUNCTION;

  // Strong detection for left, right, and center
  bool leftStrong = sensorValues[0] > LINE_THRESHOLD && sensorValues[1] > LINE_THRESHOLD && sensorValues[2] > LINE_THRESHOLD;
  bool rightStrong = sensorValues[NUM_SENSORS - 1] > LINE_THRESHOLD && sensorValues[NUM_SENSORS - 2] > LINE_THRESHOLD && sensorValues[-3] > LINE_THRESHOLD;
  bool centerStrong = sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD;

  if (leftStrong && rightStrong && centerStrong)
    return T_JUNCTION;
  if (leftStrong && centerStrong && !rightStrong)
    return LEFT_JUNCTION;
  if (rightStrong && centerStrong && !leftStrong)
    return RIGHT_JUNCTION;
  return NO_JUNCTION;
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  // Button setup
  pinMode(BUTTON_CALIBRATE, INPUT_PULLDOWN);
  pinMode(BUTTON_START, INPUT_PULLDOWN);

  Serial.println("=== Line Following Robot Starting ===");
  Serial.println("Ready: robot will line-trace and immediately turn on left/right junctions.");

  // QTR setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, NUM_SENSORS);
  delay(1000);

  // Wait for calibration button
  // waitForButton(BUTTON_CALIBRATE, "Press CALIBRATE button to calibrate sensors...");
  calibrateSensors();

  Serial.println("Calibration complete! Press START button to begin line following...");
  // Start button logic moved to loop()
  Serial.println("Robot ready! Waiting for START button...\n");
}

void loop()
{
  // if (!lineTracingEnabled) {
  //   stopMotors(); // Ensure motors are always stopped
  //   if (digitalRead(BUTTON_START) == HIGH) {
  //     delay(200); // Debounce
  //     while (digitalRead(BUTTON_START) == HIGH) delay(10); // Wait for release
  //     delay(200); // Debounce
  //     lineTracingEnabled = true;
  //     Serial.println("START button pressed! Beginning line following...");
  //   }
  //   return; // Do nothing else
  // }

  performLineFollowing();

  delay(2); // Reduced delay for faster loop
}

void stopMotors()
{
  rightMotor.brake();
  leftMotor.brake();
}

void calibrateSensors()
{
  Serial.println("Calibrating QTR sensors...");
  Serial.println("Robot will spin in place for calibration...");

  // Spin left for half the time, then right for the other half
  int totalSteps = 150;
  int halfSteps = totalSteps / 2;
  int spinSpeed = 100;

  for (int i = 0; i < totalSteps; i++)
  {
    if (i < halfSteps)
    {
      // Spin left
      rightMotor.drive(spinSpeed);
      leftMotor.drive(-spinSpeed);
    }
    else
    {
      // Spin right
      rightMotor.drive(-spinSpeed);
      leftMotor.drive(spinSpeed);
    }
    qtr.calibrate();
    delay(20);
  }
  stopMotors();
  Serial.println("Calibration complete!\n");
}

void executeAction(JunctionAction action)
{
  switch (action)
  {
  case FORWARD:
    rightMotor.drive(TurnSpeed);
    leftMotor.drive(TurnSpeed);
    break;
  case LEFT_90:
    rightMotor.drive(TurnSpeed);
    leftMotor.drive(0);
    break;
  case LEFT_45:
    rightMotor.drive(TurnSpeed);
    leftMotor.drive(-TurnSpeed / 2);
    break;
  case RIGHT_90:
    leftMotor.drive(TurnSpeed);
    rightMotor.drive(0);
    break;
  case RIGHT_45:
    leftMotor.drive(TurnSpeed);
    rightMotor.drive(-TurnSpeed / 2);
    break;
  case STOP:
    stopMotors();
    break;
  case IGNORE:
    break;
  }
}

void performLineFollowing()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  // Junction ignore logic: while active, just follow the line.
  if (junctionIgnoreActive)
  {
    if (millis() - junctionIgnoreStartTime <= JUNCTION_IGNORE_DURATION)
    {
      int error = static_cast<int>(position) - CENTER_POS;
      int correction = static_cast<int>(Kp * error + Kd * (error - lastError));
      lastError = error;
      int leftSpeed = BaseSpeed + correction;
      int rightSpeed = BaseSpeed - correction;
      leftSpeed = constrain(leftSpeed, 0, MaxSpeed);
      rightSpeed = constrain(rightSpeed, 0, MaxSpeed);
      rightMotor.drive(rightSpeed);
      leftMotor.drive(leftSpeed);
      return;
    }
    else
    {
      junctionIgnoreActive = false;
    }
  }

  // Detect junction type and react immediately for left/right junctions.
  JunctionType detected = detectJunctionType();
  if (detected == T_JUNCTION)
  {
    Serial.println("\n>>> T JUNCTION DETECTED: executing right turn");
    delay(50);
    turnUntilLine(RIGHT_90);
    junctionIgnoreActive = true;
    junctionIgnoreStartTime = millis();
    return;
  }
  else if (detected == RIGHT_JUNCTION)
  {
    Serial.println("\n>>> RIGHT JUNCTION DETECTED: executing right turn");
    delay(50);
    turnUntilLine(RIGHT_90);
    junctionIgnoreActive = true;
    junctionIgnoreStartTime = millis();
    return;
  }
  else if (detected == LEFT_JUNCTION)
  {
    delay(50);
    Serial.println("\n>>> LEFT JUNCTION DETECTED: executing left turn");
    turnUntilLine(LEFT_90);
    junctionIgnoreActive = true;
    junctionIgnoreStartTime = millis();
    return;
  }
  // Regular PID line following with speed scaling
  int error = static_cast<int>(position) - CENTER_POS;

  int correction = static_cast<int>(Kp * error + Kd * (error - lastError));
  lastError = error;

  int leftSpeed = BaseSpeed + correction;
  int rightSpeed = BaseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, MaxSpeed);
  rightSpeed = constrain(rightSpeed, 0, MaxSpeed);

  rightMotor.drive(rightSpeed);
  leftMotor.drive(leftSpeed);
}
// Robust turn-until-line function for junctions
void turnUntilLine(JunctionAction action)
{
  // Start the turn
  if (action == LEFT_90)
  {
    rightMotor.drive(TurnSpeed);
    leftMotor.drive(-TurnSpeed);
  }
  else if (action == RIGHT_90)
  {
    leftMotor.drive(TurnSpeed);
    rightMotor.drive(-TurnSpeed);
  }
  else if (action == LEFT_45)
  {
    rightMotor.drive(TurnSpeed);
    leftMotor.drive(-TurnSpeed / 2);
  }
  else if (action == RIGHT_45)
  {
    leftMotor.drive(TurnSpeed);
    rightMotor.drive(-TurnSpeed / 2);
  }
  {
    return;
  }

  const unsigned long MIN_TURN_TIME = 120; // ms
  unsigned long turnStart = millis();
  // Minimum turn time to avoid false triggers
  while (millis() - turnStart < MIN_TURN_TIME)
  {
    delay(2);
  }
  // Now look for the new line with both center sensors
  while (true)
  {
    qtr.readLineBlack(sensorValues);
    bool centerOnLine = (sensorValues[3] > LINE_THRESHOLD && sensorValues[4] > LINE_THRESHOLD);
    if (centerOnLine)
      break;
    if (millis() - turnStart > 500)
      break; // safety timeout
    delay(2);
  }
  stopMotors();
}
