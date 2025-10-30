#ifndef PID_CONFIG_H
#define PID_CONFIG_H

/**
 * @brief PID configuration and motor speed settings for line following.
 *
 * This struct holds the PID constants and motor speed parameters
 * used by the line tracing algorithm.
 */
struct PID_CONFIG
{
    float Kp;            ///< Proportional gain constant
    float Ki;            ///< Integral gain constant
    float Kd;            ///< Derivative gain constant
    uint8_t MAX_SPD;     ///< Maximum motor speed
    uint8_t BASE_SPD;    ///< Base motor speed (normal forward speed)
    uint8_t TURN_SPD;    ///< Speed used when turning
    uint16_t CENTER_POS; ///< Center position value for sensor readings
};

#endif