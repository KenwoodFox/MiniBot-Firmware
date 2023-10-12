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

    int curspeed = 0; // 0 to 255
    long int lastPulse = 0;

    for (;;) // Run forever
    {
        // Test motor writing
        analogWrite(config->_pwmPin, curspeed); // PWM out
        digitalWrite(config->_aPin, config->inverted ? HIGH : LOW);
        digitalWrite(config->_bPin, config->inverted ? LOW : HIGH);

        // Should be able to read from that pointer
        // Log.infoln("%s: Motor pulse is %d.", pcTaskGetName(NULL), *config->encPtr);

        Log.infoln("%s: (%l, %d)", pcTaskGetName(NULL), (*config->encPtr - lastPulse) * 10, curspeed);

        curspeed++;
        lastPulse = *config->encPtr;

        if (curspeed == 255)
        {
            curspeed = 0;
        }

        // PID response here

        /**
         * External things we need
         *
         * - Target setpoint, could use a global var in memory or possibly experiment with semaphores!
         * - External encoder hookup, polling kinda sucks so lets just use the built in. (mega only runs at like, 8mhz)
         * - Dual tasks for each side. Use one task def, but spawn it twice in setup. Twice the memory, half the headache.
         */

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}