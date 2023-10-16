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

// Controls
const long int eventsTo90 = 600;
double starSetpoint = 0.0;
double portSetpoint = 0.0;

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
    pinMode(STAR_ENC, INPUT_PULLUP);
    pinMode(PORT_ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STAR_ENC), isrHandlerStar, FALLING);
    attachInterrupt(digitalPinToInterrupt(PORT_ENC), isrHandlerPort, FALLING);

    // Setup PID Configs
    static PIDConfig starPIDConfig = {true, 2.0, 1.0, 0.9, INA1A, INA2A, MotorPWM_A, &starPulse, &starSetpoint};
    static PIDConfig portPIDConfig = {false, 2.0, 1.0, 0.9, INA1B, INA2B, MotorPWM_B, &portPulse, &portSetpoint};

    // Spawn the same task template twice, putting two copies in memory
    xTaskCreate(
        TaskPID,
        "StarPID",
        160,
        &starPIDConfig,
        2,
        &TaskStarPID_Handler);

    xTaskCreate(
        TaskPID,
        "PortPID",
        160,
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

    // Pause for a bit
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);

    // Task will never return from here
    for (;;)
    {
        // Setpoints
        starSetpoint += 500;
        portSetpoint += 500;

        /**
         * == NEW PLAN FOR NEXT TIME ==
         *
         * Use an arc, use velocity pid mode too. Sweep a nice smooth arc to the finish and stop when the
         * odometry says we've traveled for one half a circle!
         */

        rgb.setPixelColor(0, 1 * maxBrigh, 0, 0);
        rgb.show();

        xTaskDelayUntil(&prevTime, 2000 / portTICK_PERIOD_MS);

        starSetpoint += eventsTo90;

        rgb.setPixelColor(0, 0, 1 * maxBrigh, 0);
        rgb.show();

        xTaskDelayUntil(&prevTime, 1500 / portTICK_PERIOD_MS);

        // Go to sleep (await cleanup)
        xTaskDelayUntil(&prevTime, 100 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    // Nothing in loop!
    ;
}