#ifndef BOT_8S_MOTORS_H
#define BOT_8S_MOTORS_H

#include "Arduino.h"
#include "hardwareMapping.h"

// Holds the hardware pin configuration for a motor. PWN, DIR1, DIR2, and OFFSET
struct MotorPins
{
    uint8_t PIN_PWM;  // control motor speed
    uint8_t PIN_DIR1; // direction control pin 1
    uint8_t PIN_DIR2; // direction control pin 2
    int8_t OFFSET;    // 1 for normal, -1 if motor is reversed
};

MotorPins createMotor(
    uint8_t PWM_PIN, uint8_t DIR1_PIN, uint8_t DIR2_PIN, int8_t OFFSET);

// initializes motorspins as output
void initMotors(const MotorPins &MOTOR_A, const MotorPins &MOTOR_B, const uint8_t STBY);

// motor movements
void motorForward(const MotorPins &motor, const uint8_t speed);
void motorBackward(const MotorPins &motor, const uint8_t speed);
void motorStop(const MotorPins &motor);
void motorCoast(const uint8_t STBY);
void motorResume(const uint8_t STBY);
void motorSpinInPlace(const MotorPins &motorA, const MotorPins &motorB, const uint8_t speed, bool reverse = false);

#endif