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

// Task Handlers
TaskHandle_t TaskLEDs_Handler;
TaskHandle_t TaskButtons_Handler;

// Config
const int maxBrigh = 20; // Joe's eyes hurt!

// Semaphores/Flags/User
float uIntens[3] = {0.5, 0.5, 0.5};
Color chosenColor = RED;
int lastBrightness = 0;
float curBrightness = 1.0;

// Hardware Objects
Bounce2::Button userButton1 = Bounce2::Button();
Bounce2::Button userButton2 = Bounce2::Button();
Bounce2::Button userButton3 = Bounce2::Button();
Adafruit_NeoPixel rgb(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Prototypes
void TaskLED(void *pvParameters);
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
        TaskLED,            // A pointer to this task in memory
        "TaskLED",          // A name just for humans
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

    // Setup pot
    pinMode(BRIGHTNESS_PIN, INPUT);

    // Setup objects
    rgb.begin();

    for (;;)
    {
        if (userButton1.pressed())
        {
            // Runs when physically pressed
            uIntens[chosenColor] = uIntens[chosenColor] + 0.1 <= 1.0 ? uIntens[chosenColor] + 0.1 : 1; // Updates and clamps
            Log.verboseln("User button one pressed. Increasing component %d to %F", chosenColor, uIntens[chosenColor]);
        }

        if (userButton2.pressed())
        {
            // Runs when physically pressed
            uIntens[chosenColor] = uIntens[chosenColor] - 0.1 >= 0.0 ? uIntens[chosenColor] - 0.1 : 0; // Updates and clamps
            Log.verboseln("User button two pressed. Decreasing component %d to %F", chosenColor, uIntens[chosenColor]);
        }

        if (userButton3.pressed())
        {
            // UB 3 changes the currently selected color.
            chosenColor++;
            Log.verboseln("Chosen color is now %s", chosenColor == RED ? "red" : chosenColor == GREEN ? "green"
                                                                                                      : "blue");
        }

        // Update buttons
        userButton1.update();
        userButton2.update();
        userButton3.update();

        int _brightness = analogRead(BRIGHTNESS_PIN);
        if (abs(_brightness - lastBrightness) > 10)
        {
            lastBrightness = _brightness;
            curBrightness = _brightness / 1024.0;
            Log.verboseln("Brightness is now %F", curBrightness);
        }

        // Update frequency of this task
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

void TaskLED(void *pvParameters)
{
    (void)pvParameters;
    // Setup here
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    // Prev time
    TickType_t prevTime;

    // We are required to run this inital task for 4 seconds.
    Log.infoln("Beginning bootup sequence.");                       // Log we're booting up
    rgb.setPixelColor(0, 1 * maxBrigh, 0, 0);                       // Set Red
    rgb.show();                                                     // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);          // Sleep for 1 sec
    rgb.setPixelColor(0, 0, 1 * maxBrigh, 0);                       // Green
    rgb.show();                                                     // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);          // Sleep for 1 sec
    rgb.setPixelColor(0, 0, 0, 1 * maxBrigh);                       // Blue
    rgb.show();                                                     // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);          // Sleep for 1 sec
    rgb.setPixelColor(0, 1 * maxBrigh, 1 * maxBrigh, 1 * maxBrigh); // White
    rgb.show();                                                     // Push
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS);          // Sleep for 1 sec
    Log.infoln("Bootup done");                                      // Done

    // Task will never return from here
    for (;;)
    {
        // Do this literally every single time
        rgb.setPixelColor(0,                                        // Address (always 0 on this board)
                          uIntens[0] * (maxBrigh * curBrightness),  // Red component
                          uIntens[1] * (maxBrigh * curBrightness),  // Green component
                          uIntens[2] * (maxBrigh * curBrightness)); // Blue component
        rgb.show();                                                 // Push

        // Go to sleep (await cleanup)
        xTaskDelayUntil(&prevTime, 100 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    // Nothing in loop!
    ;
}
