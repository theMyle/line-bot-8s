#ifndef BOT_10S_SENSORS_H
#define BOT_10S_SENSORS_H

#include <QTRSensors.h>

void calibrateSensors(QTRSensors &qtr);
void calibrateSensorsFull(
    QTRSensors &qtr, int totalIterations = 200, int delayMs = 5,
    const MotorPins &motorA, const MotorPins &motorB, const uint8_t motorSpeed);

float calculatePosition(uint16_t *sensorValues, const float *sensorPos, const uint8_t sensorCount);

void serialDebugSensors(QTRSensors &qtr, uint16_t *sensorValues, uint8_t sensorCount, uint8_t dlay);

#endif