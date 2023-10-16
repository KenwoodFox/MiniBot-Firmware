#include <Arduino.h>
#include <ArduinoLog.h>
#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>

#include "config.h"

enum PIDMode
{
    ABSOLUTE_MODE,
    VELOCITY_MODE,
};

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

    // Mode
    PIDMode curMode = ABSOLUTE_MODE;

    // Construct a simple PID controller
    double curSpeed; // Curspeed is our input value
    double setPoint; // The target speed
    double setSpeed; // The output speed
    PID localPID(&curSpeed, &setSpeed, &setPoint, config->_p, config->_i, config->_d, DIRECT);

    setPoint = 300; // We want to target a specific RPM TODO: DELETE ME!

    long int lastPulse; // The last encoder pulse

    // Enable PID
    localPID.SetMode(AUTOMATIC);

    for (;;) // Run forever
    {
        if (curMode == VELOCITY_MODE)
        {
            // Update the current speed
            curSpeed = *config->encPtr - lastPulse;
        }
        else
        {
            // Cur "speed" is just the current setpoint
            curSpeed = *config->encPtr;
        }

        // Compute PID
        localPID.Compute();

        // Update the motor outputs
        analogWrite(config->_pwmPin, setSpeed);                     // PWM out
        digitalWrite(config->_aPin, config->inverted ? HIGH : LOW); // Direction setting
        digitalWrite(config->_bPin, config->inverted ? LOW : HIGH);

        Log.infoln("%s: Current speed is %D, output is %D", pcTaskGetName(NULL), curSpeed, setSpeed);

        if (curMode == VELOCITY_MODE)
        {
            lastPulse = *config->encPtr; // Reset the encoder absolute
        }

        vTaskDelay(40 / portTICK_PERIOD_MS); // Return in 40 ticks
    }
}