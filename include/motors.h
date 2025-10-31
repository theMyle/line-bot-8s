#ifndef BOT_8S_MOTORS_H
#define BOT_8S_MOTORS_H

#include "Arduino.h"
#include "hardwareMapping.h"

// Holds the hardware pin configuration for a motor. PWM, DIR1, DIR2, and OFFSET
struct MotorPins
{
    uint8_t PIN_PWM;  // control motor speed
    uint8_t PIN_DIR1; // direction control pin 1
    uint8_t PIN_DIR2; // direction control pin 2
    int8_t OFFSET;    // 1 for normal, -1 if motor is reversed
};

struct MotorController
{
    MotorPins LEFT_MOTOR;
    MotorPins RIGHT_MOTOR;
    uint8_t PIN_STBY;
};

MotorPins createMotor(
    uint8_t PWM_PIN, uint8_t DIR1_PIN, uint8_t DIR2_PIN, int8_t OFFSET);

// initializes motorspins as output
void initMotors(const MotorController &motors);

// single motor movements
void motorForward(const MotorPins &motor, const uint8_t speed);
void motorBackward(const MotorPins &motor, const uint8_t speed);
void motorStop(const MotorPins &motor);
void motorCoast(const uint8_t STBY);
void motorResume(const uint8_t STBY);

// motor pair movements
void driveForward(const MotorController &motors, const uint8_t speed);
void driveBackward(const MotorController &motors, const uint8_t speed);
void stopMotors(const MotorController &motors);
void moveSpinInPlace(const MotorController &motors, const uint8_t speed, bool reverse = false);

#endif