#ifndef BOT_8S_HARDWARE_MAPPING_H
#define BOT_8S_HARDWARE_MAPPING_H

// -- MOTORS --

constexpr uint8_t PIN_PWM_A = 16;    // control motor speed
constexpr uint8_t PIN_DIR_AIN1 = 13; // pin 1 for setting rotation direction
constexpr uint8_t PIN_DIR_AIN2 = 12; // pin 2 for setting rotation direction

constexpr uint8_t PIN_PWM_B = 17;    // control motor speed
constexpr uint8_t PIN_DIR_BIN1 = 18; // pin 1 for setting rotation direction
constexpr uint8_t PIN_DIR_BIN2 = 19; // pin 2 for setting rotation direction

constexpr uint8_t STBY = 23; // driver activation pin. High - enabled (control), LOW - disabled (coast)

// -- SENSORS --

constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {26, 33, 32, 35, 34, 39, 36, 25}; // 8 sensors configure left to right

// -- BUTTONS --

#endif