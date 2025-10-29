#include "motors.h"

#include "Arduino.h"
#include "hardwareMapping.h"

void initMotors(const MotorPins &MOTOR_A, const MotorPins &MOTOR_B)
{
    pinMode(MOTOR_A.PIN_PWM, OUTPUT);
    pinMode(MOTOR_B.PIN_PWM, OUTPUT);
}