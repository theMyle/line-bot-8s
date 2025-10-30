
#include <Arduino.h>
#include <QTRSensors.h>

#include "hardwareMapping.h"
#include "pidConfig.h"
#include "motors.h"
#include "sensors.h"

// sensor
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

MotorPins MOTOR_A = createMotor(PIN_PWM_A, PIN_DIR_AIN1, PIN_DIR_AIN2, -1); // right motor
MotorPins MOTOR_B = createMotor(PIN_PWM_B, PIN_DIR_BIN1, PIN_DIR_BIN2, 1);  // left motor

PID_CONFIG PID{
    10.0f, // Kp
    0.0f,  // Ki
    0.0f,  // Kd
    150,   // max
    70,    // base
    0,     // turn
    0,     // center
};

// setup things
void setup()
{
    Serial.begin(115200);
    delay(2000);

    qtr.setTypeAnalog();
    qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    qtr.releaseEmitterPins();

    initMotors(MOTOR_A, MOTOR_B, STBY);
    calibrateSensors(qtr);

    Serial.println("Ready. Reading values...");
}

unsigned long lastTime = 0;
unsigned long now = 0;
float lastError = 0;
float dt = 0;

float position;

// main loop
void loop()
{
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    if (dt <= 0)
        dt = 0.001;
    lastTime = now;

    qtr.readCalibrated(sensorValues);

    // PID
    position = calculatePosition(sensorValues, SENSOR_POS, SENSOR_COUNT);
    float error = position - PID.CENTER_POS;

    float P = PID.Kp * error;
    float D = PID.Kd * (error - lastError) / dt;
    lastError = error;

    float correction = P + D;

    int left = PID.BASE_SPD - correction;
    int right = PID.BASE_SPD + correction;

    left = constrain(left, 0, PID.MAX_SPD);
    right = constrain(right, 0, PID.MAX_SPD);

    // motorForward(MOTOR_A, right);
    // motorForward(MOTOR_B, left);

    // Serial.printf("Error: %.2f\tCorrection: %.2f\tTotal: %.2f\n", error, correction, total);
}
