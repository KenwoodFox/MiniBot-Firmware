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
#include <Adafruit_NeoPixel.h>

// Headers
#include "boardPins.h"
#include "color.h"
#include "config.h"
#include "encoder.h"

// Task Handlers
TaskHandle_t TaskLEDs_Handler;
TaskHandle_t TaskStarPID_Handler;
TaskHandle_t TaskPortPID_Handler;
Adafruit_NeoPixel rgb(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Prototypes
void TaskLED(void *pvParameters);
void TaskPID(void *pvParameters); // The nice thing about these task prototypes is we can redefine new ones using new pvparams!

void setup()
{
    // Setup serial
    Serial.begin(115200); // USB (debug)
    Serial.print("\n\n");
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.infoln("Init: Beginning user code! Version %s", REVISION);

    // Pins
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup regular tasks
    xTaskCreate(
        TaskLED,            // A pointer to this task in memory
        "LEDs",             // A name just for humans
        128,                // This stack size can be checked & adjusted by reading the Stack Highwater
        NULL,               // Parameters passed to the task function
        2,                  // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &TaskLEDs_Handler); // Task handle

    // Configure hardware inturrupts
    attachInterrupt(digitalPinToInterrupt(STAR_ENC), isrHandlerStar, FALLING);
    attachInterrupt(digitalPinToInterrupt(PORT_ENC), isrHandlerPort, FALLING);

    // Setup PID Configs
    static PIDConfig starPIDConfig = {true, 1.0, 0.1, 0.1, INA1A, INA2A, MotorPWM_A};
    static PIDConfig portPIDConfig = {false, 1.0, 0.1, 0.1, INA1B, INA2B, MotorPWM_B};

    // Spawn the same task template twice, putting two copies in memory
    xTaskCreate(
        TaskPID,
        "StarPID",
        128,
        &starPIDConfig,
        2,
        &TaskStarPID_Handler);

    xTaskCreate(
        TaskPID,
        "PortPID",
        128,
        &portPIDConfig,
        2,
        &TaskPortPID_Handler);
}

void TaskLED(void *pvParameters)
{
    (void)pvParameters;
    // Setup here
    rgb.begin();

    // Prev time
    TickType_t prevTime;

    // We are required to run this inital task for 4 seconds.
    Log.infoln("%s: Beginning bootup sequence.", pcTaskGetName(NULL)); // Log we're booting up
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);             // Sleep for 1 sec
    rgb.setPixelColor(0, 1 * maxBrigh, 0, 0);                          // Set Red
    rgb.show();                                                        // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);             // Sleep for 1 sec
    rgb.setPixelColor(0, 0, 1 * maxBrigh, 0);                          // Green
    rgb.show();                                                        // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);             // Sleep for 1 sec
    Log.infoln("%s: Bootup done", pcTaskGetName(NULL));                // Done

    // Task will never return from here
    for (;;)
    {
        // Nothing to do here.

        // Go to sleep (await cleanup)
        xTaskDelayUntil(&prevTime, 100 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    // Nothing in loop!
    ;
}