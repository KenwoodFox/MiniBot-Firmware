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
#include "color.h"
#include "config.h"
#include "encoder.h"
#include "colors.h"

// Task Handlers
TaskHandle_t TaskLEDs_Handler;
TaskHandle_t TaskStarPID_Handler;
TaskHandle_t TaskPortPID_Handler;

// Prototypes
void TaskLED(void *pvParameters);
void TaskPID(void *pvParameters); // The nice thing about these task prototypes is we can redefine new ones using new pvparams!

// Buttons
Bounce2::Button userButton1 = Bounce2::Button();

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

    // Buttons
    userButton1.attach(UB1_PIN, INPUT_PULLUP);

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
    static PIDConfig starPIDConfig = {true, 1.8, 1.0, 0.9, INA1A, INA2A, MotorPWM_A, &starPulse, &starSetpoint};
    static PIDConfig portPIDConfig = {false, 1.8, 1.0, 0.9, INA1B, INA2B, MotorPWM_B, &portPulse, &portSetpoint};

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
    setRed();
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS); // Sleep for 1 sec
    Log.infoln("%s: Bootup done", pcTaskGetName(NULL));    // Done
    setGreen();

    // Pause for a bit
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);

    // Task will never return from here
    for (;;)
    {
        /**
         * == NEW PLAN FOR NEXT TIME ==
         *
         * Use an arc, use velocity pid mode too. Sweep a nice smooth arc to the finish and stop when the
         * odometry says we've traveled for one half a circle!
         */

        // Check if run button pressed
        if (userButton1.pressed())
        {
            Log.infoln(F("%s: User button pressed, ready to go"), pcTaskGetName(NULL));

            setRed();

            /**
             * Sequence start
             */
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            portSetpoint += 120;
            Log.infoln(F("%s: Advance star by 200"), pcTaskGetName(NULL));
            vTaskDelay(1500 / portTICK_PERIOD_MS);

            starSetpoint += 450;
            portSetpoint += 540;
            Log.infoln(F("%s: Forward"), pcTaskGetName(NULL));
            vTaskDelay(1500 / portTICK_PERIOD_MS);

            portSetpoint += 320;
            Log.infoln(F("%s: Left"), pcTaskGetName(NULL));
            vTaskDelay(900 / portTICK_PERIOD_MS);

            starSetpoint += 650;
            portSetpoint += 660;
            Log.infoln(F("%s: Forward"), pcTaskGetName(NULL));
            vTaskDelay(2500 / portTICK_PERIOD_MS);

            starSetpoint += 120;
            Log.infoln(F("%s: Turn back"), pcTaskGetName(NULL));
            vTaskDelay(1500 / portTICK_PERIOD_MS);

            vTaskDelay(1500 / portTICK_PERIOD_MS);

            Log.infoln(F("%s: Done"), pcTaskGetName(NULL));
            setGreen(); // Done!
        }

        // Update all buttons
        userButton1.update();
    }
}

void loop()
{
    // Nothing in loop!
    ;
}