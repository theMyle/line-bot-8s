
#include <Arduino.h>
#include <QTRSensors.h>

#include "hardwareMapping.h"
#include "pidConfig.h"
#include "motors.h"

// sensor
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

MotorPins MOTOR_A = createMotor(PIN_PWM_A, PIN_DIR_AIN1, PIN_DIR_AIN2, 1);
MotorPins MOTOR_B = createMotor(PIN_PWM_B, PIN_DIR_BIN1, PIN_DIR_BIN2, -1);

PID_CONFIG PID{
    0.08f, // Kp
    0.0f,  // Ki
    0.2f,  // Kd
    150,   // max
    120,   // base
    80,    // turn
    2500,  // center
};

// setup things
void setup()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.releaseEmitterPins();

    initMotors(MOTOR_A, MOTOR_B, STBY);
}

// main loop
void loop()
{
    // sample bot spin
    const uint8_t speed = 20;

    motorForward(MOTOR_A, speed);
    motorBackward(MOTOR_B, speed);
    delay(3000);

    motorStop(MOTOR_A);
    motorStop(MOTOR_B);
    delay(5000);

    motorForward(MOTOR_B, speed);
    motorBackward(MOTOR_A, speed);
    delay(3000);
}