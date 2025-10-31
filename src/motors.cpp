#include "motors.h"

#include "Arduino.h"
#include "hardwareMapping.h"

// creates a MotorPins struct with given PWM, direction pins, and offset
MotorPins createMotor(
    uint8_t PWM_PIN, uint8_t DIR1_PIN, uint8_t DIR2_PIN, int8_t OFFSET)
{
    return {PWM_PIN, DIR1_PIN, DIR2_PIN, OFFSET};
};

// initializes motor pins and enables the driver
void initMotors(const MotorPins &MOTOR_A, const MotorPins &MOTOR_B, const uint8_t STBY)
{
    pinMode(MOTOR_A.PIN_PWM, OUTPUT);
    pinMode(MOTOR_A.PIN_DIR1, OUTPUT);
    pinMode(MOTOR_A.PIN_DIR2, OUTPUT);

    pinMode(MOTOR_B.PIN_PWM, OUTPUT);
    pinMode(MOTOR_B.PIN_DIR1, OUTPUT);
    pinMode(MOTOR_B.PIN_DIR2, OUTPUT);

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
}

// movements
void motorForward(const MotorPins &motor, const uint8_t speed)
{
    digitalWrite(motor.PIN_DIR1, motor.OFFSET > 0 ? HIGH : LOW);
    digitalWrite(motor.PIN_DIR2, motor.OFFSET > 0 ? LOW : HIGH);
    analogWrite(motor.PIN_PWM, speed);
}

void motorBackward(const MotorPins &motor, const uint8_t speed)
{
    digitalWrite(motor.PIN_DIR1, motor.OFFSET > 0 ? LOW : HIGH);
    digitalWrite(motor.PIN_DIR2, motor.OFFSET > 0 ? HIGH : LOW);
    analogWrite(motor.PIN_PWM, speed);
}

void motorStop(const MotorPins &motor)
{
    digitalWrite(motor.PIN_DIR1, motor.OFFSET > 0 ? HIGH : LOW);
    digitalWrite(motor.PIN_DIR2, motor.OFFSET > 0 ? HIGH : LOW);
    analogWrite(motor.PIN_PWM, 0);
}

// dissables motor driver - coasts
void motorCoast(const uint8_t STBY)
{
    digitalWrite(STBY, LOW);
}

// re-enables motor driver after coasting
void motorResume(const uint8_t STBY)
{
    digitalWrite(STBY, HIGH);
}

void motorSpinInPlace(const MotorPins &motorA, const MotorPins &motorB, const uint8_t speed, bool reverse = false)
{
    if (!reverse)
    {
        motorForward(motorA, speed);
        motorBackward(motorB, speed);
    }
    else
    {
        motorForward(motorB, speed);
        motorBackward(motorA, speed);
    }
}