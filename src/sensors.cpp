#include <QTRSensors.h>
#include <Arduino.h>

#include "sensors.h"
#include "motors.h"

void calibrateSensors(QTRSensors &qtr)
{
    // Serial.print("calibrating...");
    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
        delay(5);
        // Serial.print(". ");
    }
    // Serial.println("Done calibrating...");
}

void calibrateSensorsFull(
    QTRSensors &qtr, int totalIterations = 200, int delayMs = 5,
    const MotorPins &motorA, const MotorPins &motorB, const uint8_t motorSpeed)
{
    for (uint16_t i = 0; i < totalIterations; i++)
    {
        qtr.calibrate();
        bool reverse = i >= (totalIterations / 2);
        motorSpinInPlace(motorA, motorB, motorSpeed, reverse);
        delay(delayMs);
    }

    motorStop(motorA);
    motorStop(motorB);
}

float calculatePosition(uint16_t *sensorValues, float *sensorPos, uint8_t sensorCount)
{
    float sum = 0;
    float total = 0;

    for (int i = 0; i < sensorCount; i++)
    {
        sum += sensorValues[i] * sensorPos[i];
        total += sensorValues[i];
    }

    if (total == 0)
    {
        return -1.0;
    }
    else
    {
        return sum / total;
    }
}

void serialDebugSensors(QTRSensors &qtr, uint16_t *sensorValues, uint8_t sensorCount, uint8_t dlay)
{
    qtr.read(sensorValues);

    for (int i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }
    Serial.println();

    delay(dlay);
}