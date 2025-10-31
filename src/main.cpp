
#include <Arduino.h>
#include <QTRSensors.h>

#include "hardwareMapping.h"
#include "pidConfig.h"
#include "motors.h"
#include "sensors.h"

// sensor
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

const MotorController motorController{
    .LEFT_MOTOR = createMotor(PIN_PWM_B, PIN_DIR_BIN1, PIN_DIR_BIN2, 1),
    .RIGHT_MOTOR = createMotor(PIN_PWM_A, PIN_DIR_AIN1, PIN_DIR_AIN2, -1),
    .PIN_STBY = STBY,
};

const PID_CONFIG PID{
    .Kp = 10.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MAX_SPD = 150,
    .BASE_SPD = 70,
    .TURN_SPD = 0,
    .CENTER_POS = 0,
};

// setup things
void setup()
{
    // Serial.begin(115200);
    delay(3000);

    qtr.setTypeAnalog();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.releaseEmitterPins();

    initMotors(motorController);
    calibrateSensorsFull(qtr, 400, 5, motorController, 80);
}

unsigned long now = 0;
unsigned long lastTime = 0;
float lastError = 0;

float leftMotorFloatAccumulator = PID.BASE_SPD;
float rightMotorFloatAccumulator = PID.BASE_SPD;

// main loop
void loop()
{
    now = millis();
    float dt = (now - lastTime) / 1000.0;
    dt = (dt <= 0) ? 0.001 : dt;
    lastTime = now;

    // PID
    qtr.readCalibrated(sensorValues);
    float position = calculatePosition(sensorValues, SENSOR_POSITION_OFFSETS, SENSOR_COUNT);
    float error = position - PID.CENTER_POS;

    float P = PID.Kp * error;
    float D = PID.Kd * (error - lastError) / dt;
    lastError = error;

    float correction = P + D;
    leftMotorFloatAccumulator -= correction;
    rightMotorFloatAccumulator += correction;

    int leftCorrection = constrain((int)leftMotorFloatAccumulator, 0, PID.MAX_SPD);
    int rightCorrection = constrain((int)rightMotorFloatAccumulator, 0, PID.MAX_SPD);

    // int leftCorrection = constrain(PID.BASE_SPD - correction, 0, PID.MAX_SPD);
    // int rightCorrection = constrain(PID.BASE_SPD + correction, 0, PID.MAX_SPD);

    // apply motor
    motorForward(motorController.LEFT_MOTOR, leftCorrection);
    motorForward(motorController.RIGHT_MOTOR, rightCorrection);
}
