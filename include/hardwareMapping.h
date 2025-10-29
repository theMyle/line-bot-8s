#ifndef HARDWARE_MAPPING_H
#define HARDWARE_MAPPING_H

// -- MOTORS --

// Holds the hardware pin configuration for a motor. PWN, DIR1, DIR2, and OFFSET
struct MotorPins
{
    uint8_t PIN_PWM;  // control motor speed
    uint8_t PIN_DIR1; // direction control pin 1
    uint8_t PIN_DIR2; // direction control pin 2
    bool OFFSET;      // 1 for normal, -1 if motor is reversed
};

const MotorPins MOTOR_A = {
    16, // PWM
    13, // DIR1
    12, // DIR2
    1,  // OFFSET
};
const MotorPins MOTOR_B = {
    17, // PWN
    18, // DIR1
    19, // DIR2
    -1, // OFFSET
};

const uint8_t STBY = 23; // driver activation pin. High - enabled (control), LOW - disabled (coast)

// -- SENSORS --

const uint8_t SENSOR_COUNT = 8;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {26, 33, 32, 35, 34, 39, 36, 25};

// -- BUTTONS --

#endif