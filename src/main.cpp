
#include <Arduino.h>
#include <QTRSensors.h>

#include "hardwareMapping.h"
#include "pidConfig.h"
#include "motors.h"

// sensor
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// setup things
void setup()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.releaseEmitterPins();

    initMotors(MOTOR_A, MOTOR_B);
}

// main loop
void loop()
{
}