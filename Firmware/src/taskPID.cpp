#include <Arduino.h>
#include <ArduinoLog.h>
#include <Arduino_FreeRTOS.h>

#include "config.h"

/**
 * @brief INA based PID motor controller.
 *
 * @param pvParameters A PIDConfig object.
 * @see PIDConfig
 * @author Joe
 */
void TaskPID(void *pvParameters)
{
    PIDConfig *config = (PIDConfig *)pvParameters; // We're going to cast the pointer this task was created with, to a config struct.
    // Setup here
    Log.infoln("%s: Ready.", pcTaskGetName(NULL)); // Log we're booting up
    Log.infoln("%s: Using INA pin %d, PID values are %D, %D, %D", pcTaskGetName(NULL), config->_aPin, config->_p, config->_i, config->_d);

    // Motor driver pins
    pinMode(config->_aPin, OUTPUT);
    pinMode(config->_bPin, OUTPUT);
    pinMode(config->_pwmPin, OUTPUT);

    // Init/test
    /** psudo: while i < maxImpulse (where max impulse is the maximum test impulse)
     *
     */

    int targetSpeed = 150; // We want to target a specific RPM TODO: DELETE ME!

    long int curSpeed = 0;  // The current RPS
    long int lastSpeed = 0; // The last speed (used for calculating D)
    long int lastPulse = 0;

    int setSpeed = 0; // The current setpoint

    for (;;) // Run forever
    {
        // Test motor writing
        analogWrite(config->_pwmPin, setSpeed);                     // PWM out
        digitalWrite(config->_aPin, config->inverted ? HIGH : LOW); // Direction setting
        digitalWrite(config->_bPin, config->inverted ? LOW : HIGH);

        curSpeed = (*config->encPtr - lastPulse) * 10; // (the 10 is because this loop resumes every 100ms, it could be run faster!)

        Log.infoln("%s: Current speed is %l, output is %d", pcTaskGetName(NULL), curSpeed, setSpeed);

        // Compute PID response
        long int _err = curSpeed - targetSpeed; // The magnitude of the error
        // setSpeed += _err * config->_i;                     // I grows slowly over time in the magnitude of the err
        setSpeed = _err * config->_p; // The p response
        // setSpeed -= ((setSpeed - lastSpeed) * config->_d); // d clamps the response based on last response

        lastPulse = *config->encPtr; // Reset the encoder absolute
        lastSpeed = curSpeed;        // Update last speed

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}