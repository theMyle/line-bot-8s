#ifndef BOT_10S_HARDWARE_MAPPING_H
#define BOT_10S_HARDWARE_MAPPING_H

// -- MOTORS --

constexpr uint8_t PIN_PWM_A = 16;    // control motor speed
constexpr uint8_t PIN_DIR_AIN1 = 13; // pin 1 for setting rotation direction
constexpr uint8_t PIN_DIR_AIN2 = 12; // pin 2 for setting rotation direction

constexpr uint8_t PIN_PWM_B = 17;    // control motor speed
constexpr uint8_t PIN_DIR_BIN1 = 18; // pin 1 for setting rotation direction
constexpr uint8_t PIN_DIR_BIN2 = 19; // pin 2 for setting rotation direction

constexpr uint8_t STBY = 23; // driver activation pin. High - enabled (control), LOW - disabled (coast)

// -- SENSORS --

// constexpr uint8_t SENSOR_COUNT = 8;
// constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {26, 33, 32, 35, 34, 39, 36, 25}; // 8 sensors configure left to right

// new sensors?

constexpr uint8_t SENSOR_COUNT = 10;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {39, 36, 25, 27, 14, 34, 35, 32, 33, 26}; // 10 sensors configure left to right

// constexpr float SENSOR_POS[SENSOR_COUNT] = {0, 16.5, 33.0, 49.5, 56.4, 64.0, 70.9, 87.4, 103.9, 120.4};

constexpr float SENSOR_POS[SENSOR_COUNT] = {-7.5, -5.5, -3.5, -1.5, -0.5, 0.5, 1.5, 3.5, 5.5, 7.5}; // sensor value based physical distance

// -- BUTTONS --

#endif