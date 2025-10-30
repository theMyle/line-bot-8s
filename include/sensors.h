#ifndef BOT_10S_SENSORS_H
#define BOT_10S_SENSORS_H

#include <QTRSensors.h>

void calibrateSensors(QTRSensors &qtr);
float calculatePosition(uint16_t *sensorValues, const float *sensorPos, const uint8_t sensorCount);

void serialDebugSensors(QTRSensors &qtr, uint16_t *sensorValues, uint8_t sensorCount, uint8_t dlay);

#endif