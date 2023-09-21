/**
 * @file main.cpp
 * @author Team 7
 * @brief Source code for MiniBot
 */

// Libs
#include <Bounce2.h>
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Arduino_FreeRTOS.h>

// Headers
#include "boardPins.h"

// Task Handlers
TaskHandle_t TaskLEDs_Handler;
TaskHandle_t TaskButtons_Handler;

// Semaphores/Flags
bool toggleRed = false;    // True when toggle red
bool toggleGreen = false;  // True when toggle green
const int msOn = 5 * 1000; // Time to remain on when latched
long int latchTime = 0;    // True when latched (only unlatch when time expires)

// Hardware Objects
Bounce2::Button userButton1 = Bounce2::Button();
Bounce2::Button userButton2 = Bounce2::Button();
Bounce2::Button userButton3 = Bounce2::Button();

// Prototypes
void TaskLEDs(void *pvParameters);
void TaskButtons(void *pvParameters);
bool isLatched();

void setup()
{
    // Setup serial
    Serial.begin(115200); // USB (debug)
    Serial.print("\n\n");
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.infoln("Beginning user code! Version %s", REVISION);

    // Pins
    pinMode(STAT_LED, OUTPUT);

    // Setup tasks
    xTaskCreate(
        TaskLEDs,           // A pointer to this task in memory
        "TaskLEDs",         // A name just for humans
        128,                // This stack size can be checked & adjusted by reading the Stack Highwater
        NULL,               // Parameters passed to the task function
        2,                  // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &TaskLEDs_Handler); // Task handle

    xTaskCreate(
        TaskButtons,
        "TaskButtons",
        128,
        NULL,
        2,
        &TaskButtons_Handler);
}

void TaskButtons(void *pvParameters)
{
    (void)pvParameters;

    // Setup buttons
    userButton1.attach(UB1, INPUT_PULLUP);
    userButton2.attach(UB2, INPUT_PULLUP);
    userButton3.attach(UB3, INPUT_PULLUP);

    for (;;)
    {
        if (userButton1.pressed())
        {
            // Runs when physically pressed
            Log.verboseln("User button one pressed. Toggle is %d", toggleGreen);
            toggleGreen = !toggleGreen; // Flip!
        }

        toggleRed = !userButton2.isPressed(); // Oneliner!

        if (userButton3.pressed())
        {
            // Could use a semaphore here.
            Log.verboseln("User button three pressed. Latch was %d, latch was %d", latchTime, isLatched());
            latchTime = millis(); // Come back to this later (for set expire)
        }

        // Update buttons
        userButton1.update();
        userButton2.update();
        userButton3.update();

        // Update frequency of this task
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

bool isLatched()
{
    return latchTime + msOn >= millis();
}

void TaskLEDs(void *pvParameters)
{
    (void)pvParameters;
    // Setup here
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    // Prev time
    TickType_t prevTime;

    // We are required to run this inital task for 3 seconds.
    Log.infoln("Flashing at 5hz for 3 seconds...");
    for (uint8_t i = 0; i < 3; i++)
    {
        // We must flash at 5hz
        uint8_t _freq = 5;
        for (uint8_t j = 0; j < _freq * 2; j++)
        {
            digitalWrite(GREEN_LED, j % 2 == 0 || toggleGreen);
            digitalWrite(RED_LED, j % 2 == 0 || toggleRed);

            xTaskDelayUntil(&prevTime, (1000 / (_freq * 2)) / portTICK_PERIOD_MS);
        }
    }
    Log.infoln("Entering LED Standby");

    // Task will never return from here
    for (;;)
    {
        // Lights go off
        digitalWrite(GREEN_LED, toggleGreen || isLatched());
        digitalWrite(RED_LED, toggleRed || isLatched());

        // Go to sleep (await cleanup)
        xTaskDelayUntil(&prevTime, 100 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    // Nothing in loop!
    ;
}
