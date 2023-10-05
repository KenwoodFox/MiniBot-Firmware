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

// Task Handlers
TaskHandle_t TaskLEDs_Handler;
TaskHandle_t TaskStarPID_Handler;
TaskHandle_t TaskPortPID_Handler;
Adafruit_NeoPixel rgb(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Prototypes
void TaskLED(void *pvParameters);
void TaskPID(void *pvParameters); // The nice thing about these task prototypes is we can redefine new ones using new pvparams!

// TODO: Move this somewhere else
struct PIDConfig
{
    // General
    bool inverted;

    // PID Tuning Values
    double _p;
    double _i;
    double _d;

    // Motor Controller Pins
    uint8_t _aPin;
    uint8_t _bPin;
    uint8_t _pwmPin;
};

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

void TaskPID(void *pvParameters)
{
    PIDConfig *config = (PIDConfig *)pvParameters; // We're going to cast the pointer this task was created with, to a config struct.
    // Setup here
    Log.infoln("%s: Ready.", pcTaskGetName(NULL)); // Log we're booting up
    Log.infoln("%s: Using INA pin %d, PID values are %D, %D, %D", pcTaskGetName(NULL), config->_aPin, config->_p, config->_i, config->_d);

    pinMode(config->_aPin, OUTPUT);
    pinMode(config->_bPin, OUTPUT);
    pinMode(config->_pwmPin, OUTPUT);

    for (;;) // Run forever
    {
        // Test motor writing
        analogWrite(config->_pwmPin, 90); // PWM out
        digitalWrite(config->_aPin, config->inverted ? HIGH : LOW);
        digitalWrite(config->_bPin, config->inverted ? LOW : HIGH);

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

// void TaskButtons(void *pvParameters)
// {
//     (void)pvParameters;

//     // Setup buttons
//     userButton1.attach(UB1, INPUT_PULLUP);
//     userButton2.attach(UB2, INPUT_PULLUP);
//     userButton3.attach(UB3, INPUT_PULLUP);

//     // Setup pot
//     pinMode(BRIGHTNESS_PIN, INPUT);

//     // Setup objects
//     rgb.begin();

//     for (;;)
//     {
//         if (userButton1.pressed())
//         {
//             // Runs when physically pressed
//             uIntens[chosenColor] = uIntens[chosenColor] + 0.1 <= 1.0 ? uIntens[chosenColor] + 0.1 : 1; // Updates and clamps
//             Log.verboseln("%s: UB1 Pressed, increase color %d to %F", pcTaskGetName(NULL), chosenColor, uIntens[chosenColor]);
//         }

//         if (userButton2.pressed())
//         {
//             // Runs when physically pressed
//             uIntens[chosenColor] = uIntens[chosenColor] - 0.1 >= 0.0 ? uIntens[chosenColor] - 0.1 : 0; // Updates and clamps
//             Log.verboseln("%s: UB2 Pressed, decrease color %d to %F", pcTaskGetName(NULL), chosenColor, uIntens[chosenColor]);
//         }

//         if (userButton3.pressed())
//         {
//             // UB 3 changes the currently selected color.
//             chosenColor++;
//             Log.verboseln("%s: Chosen color is now %s", pcTaskGetName(NULL), chosenColor == RED ? "red" : chosenColor == GREEN ? "green"
//                                                                                                                                : "blue");
//         }

//         // Update buttons
//         userButton1.update();
//         userButton2.update();
//         userButton3.update();

//         int _brightness = analogRead(BRIGHTNESS_PIN);
//         if (abs(_brightness - lastBrightness) > 10)
//         {
//             lastBrightness = _brightness;
//             curBrightness = _brightness / 1024.0;
//             Log.verboseln("%s: Brightness is now %F", pcTaskGetName(NULL), curBrightness);
//         }

//         // Update frequency of this task
//         vTaskDelay(40 / portTICK_PERIOD_MS);
//     }
// }

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