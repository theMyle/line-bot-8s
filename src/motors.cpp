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
void initMotors(const MotorController &motors)
{
    pinMode(motors.LEFT_MOTOR.PIN_PWM, OUTPUT);
    pinMode(motors.LEFT_MOTOR.PIN_DIR1, OUTPUT);
    pinMode(motors.LEFT_MOTOR.PIN_DIR2, OUTPUT);

    pinMode(motors.RIGHT_MOTOR.PIN_PWM, OUTPUT);
    pinMode(motors.RIGHT_MOTOR.PIN_DIR1, OUTPUT);
    pinMode(motors.RIGHT_MOTOR.PIN_DIR2, OUTPUT);

    pinMode(motors.PIN_STBY, OUTPUT);
    digitalWrite(motors.PIN_STBY, HIGH);
}

// single motor movements
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

// motor pair movements

// drive both wheels forward
void driveForward(const MotorController &motors, const uint8_t speed)
{
    motorForward(motors.LEFT_MOTOR, speed);
    motorForward(motors.RIGHT_MOTOR, speed);
};

// drive both wheels backward
void driveBackward(const MotorController &motors, const uint8_t speed)
{
    motorBackward(motors.LEFT_MOTOR, speed);
    motorBackward(motors.RIGHT_MOTOR, speed);
};

// drive both wheels the opposite direction
void motorSpinInPlace(const MotorController &motors, const uint8_t speed, bool reverse = false)
{
    if (!reverse)
    {
        motorForward(motors.LEFT_MOTOR, speed);
        motorBackward(motors.RIGHT_MOTOR, speed);
    }
    else
    {
        motorForward(motors.RIGHT_MOTOR, speed);
        motorBackward(motors.LEFT_MOTOR, speed);
    }
}

// stop both motors
void stopMotors(const MotorController &motors)
{
    motorStop(motors.LEFT_MOTOR);
    motorStop(motors.RIGHT_MOTOR);
};