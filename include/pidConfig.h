#ifndef PID_CONFIG_H
#define PID_CONFIG_H

struct PID_CONFIG
{
    float Kp;
    float Ki;
    float Kd;
    uint8_t MAX_SPD;
    uint8_t BASE_SPD;
    uint8_t TURN_SPD;
    uint16_t CENTER_POS;
};

// PID config
const PID_CONFIG PID{
    0.08f, // Kp
    0.0f,  // Ki
    0.2f,  // Kd
    150,   // max
    120,   // base
    80,    // turn
    2500,  // center
};

#endif