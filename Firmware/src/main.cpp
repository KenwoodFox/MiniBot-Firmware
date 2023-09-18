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

// Hardware Objects
Bounce2::Button userButton1 = Bounce2::Button();

// Prototypes
void TaskLEDs(void *pvParameters);

void setup()
{
    // Setup serial
    delay(200);
    Serial.begin(115200);
    Serial.println(REVISION);

    // Pins
    pinMode(STAT_LED, OUTPUT);

    // Buttons
    userButton1.attach(UB1, INPUT_PULLUP);

    // Setup tasks
    xTaskCreate(
        TaskLEDs,           // A pointer to this task in memory
        "TaskLEDs",         // A name just for humans
        128,                // This stack size can be checked & adjusted by reading the Stack Highwater
        NULL,               // Parameters passed to the task function
        2,                  // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &TaskLEDs_Handler); // Task handle
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
    for (uint8_t i = 0; i < 3; i++)
    {
        // We must flash at 5hz
        uint8_t _freq = 5;
        for (uint8_t j = 0; j < _freq * 2; j++)
        {
            digitalWrite(GREEN_LED, j % 2 == 0);
            digitalWrite(RED_LED, j % 2 == 0);

            xTaskDelayUntil(&prevTime, (1000 / (_freq * 2)) / portTICK_PERIOD_MS);
        }
    }

    for (;;) // Task will never return
    {
        // Lights go off
        digitalWrite(GREEN_LED, false);
        digitalWrite(RED_LED, false);

        // Go to sleep (await cleanup)
        xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    // Flash LED
    digitalWrite(STAT_LED, HIGH);
    delay(1000);
    digitalWrite(STAT_LED, LOW);
    delay(1000);
}
