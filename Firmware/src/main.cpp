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
#include <Servo.h>
#include <HCSR04.h>

// Headers
#include "boardPins.h"
#include "color.h"
#include "config.h"
#include "encoder.h"
#include "colors.h"

// Task Handlers
TaskHandle_t TaskNav_Handler;
TaskHandle_t TaskStarPID_Handler;
TaskHandle_t TaskPortPID_Handler;
TaskHandle_t TaskModem_Handler;

// Prototypes
void TaskNavigate(void *pvParameters);
void TaskModem(void *pvParameters);
void TaskPID(void *pvParameters); // The nice thing about these task prototypes is we can redefine new ones using new pvparams!

// Buttons
Bounce2::Button userButton1 = Bounce2::Button();

// Sensors
HCSR04 sonar(TRIG_PIN, ECHO_PIN);

// Controls
const long int eventsTo90 = 600;
double starSetpoint = 0.0;
double portSetpoint = 0.0;

// Guo wants us to create a field-map,
// we'll use 128 8 bit bytes (to start,
// we could replace this with something more efficent!)
const uint8_t occupationMapDensity = 40;
float occupationMap[occupationMapDensity];
Servo sonarServo;
bool occupationNotReady = true;

void setup()
{
    // Setup serial
    Serial.begin(115200); // USB (debug)
    Serial.print("\n\n");
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.infoln("Init: Beginning user code! Version %s", REVISION);

    // Setup comms
    Serial1.begin(38400); // Bluetooth module

    // Pins
    pinMode(LED_BUILTIN, OUTPUT);

    // Buttons
    userButton1.attach(UB1_PIN, INPUT_PULLUP);

    // Bluetooth Radio modem
    Serial1.begin(38400); // We can use one of the hardware serial devices.

    // Sonar system
    sonarServo.attach(SERVO1_PIN); // Pin 51 on board
    /* TOF reflector sensor here */

    // Setup regular tasks
    xTaskCreate(
        TaskNavigate,      // A pointer to this task in memory
        "NAV",             // A name just for humans
        200,               // This stack size can be checked & adjusted by reading the Stack Highwater
        NULL,              // Parameters passed to the task function
        2,                 // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        &TaskNav_Handler); // Task handle

    xTaskCreate(
        TaskModem,
        "Modem",
        160,
        NULL,
        2,
        &TaskModem_Handler);

    // Configure hardware inturrupts
    pinMode(STAR_ENC, INPUT_PULLUP);
    pinMode(PORT_ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STAR_ENC), isrHandlerStar, FALLING);
    attachInterrupt(digitalPinToInterrupt(PORT_ENC), isrHandlerPort, FALLING);

    // Setup PID Configs
    static PIDConfig starPIDConfig = {false, 18.0, 5.0, 0.9, INA1B, INA2B, MotorPWM_B, &starPulse, &starSetpoint};
    static PIDConfig portPIDConfig = {true, 18.0, 5.0, 0.9, INA1A, INA2A, MotorPWM_A, &portPulse, &portSetpoint};

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

void TaskModem(void *pvParameters)
{
    (void)pvParameters;

    // Prev time
    TickType_t prevTime;

    char buf[30]; // Message buffer

    // Wait
    xTaskDelayUntil(&prevTime, 1000 / portTICK_PERIOD_MS); // Sleep for 1 sec

    // Check if AT mode
    // Try enter AT mode
    Serial1.print("AT\n\r");                             // Send AT
    xTaskDelayUntil(&prevTime, 60 / portTICK_PERIOD_MS); // Sleep for 60ms
    Serial1.readBytes(buf, 4);
    if (strcmp(buf, "OK\n\r"))
    {
        Log.warningln("Modem is in AT mode! Uploading config...");
        Serial1.print("AT+NAME=Group7_SamJoe\n\r");
        Serial1.print("AT+PIN=1234\n\r");
    }

    // Done
    Log.infoln("%s: Ready", pcTaskGetName(NULL));

    for (;;)
    {
        // Start by checking the buf
        if (Serial1.available() > 0)
        {
            Serial.println(Serial1.read());
        }

        while (Serial.available() > 0)
        {
            Serial1.print(Serial.read());
        }
    }
}

void TaskNavigate(void *pvParameters)
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

    // Memory
    uint8_t lastSweep = 0;      // Used to keep track of last sweep pos
    uint8_t sweepUp = true;     // If we're sweeping up
    occupationMap[0] = 9999.99; // Prepopulate

    // Task will never return from here
    for (;;)
    {
        // Check if run button pressed
        if (userButton1.pressed())
        {
            Log.infoln("%s: User button pressed, ready to go", pcTaskGetName(NULL));
            setRed();

            // Inital just kinda scoot foward
            starSetpoint = 5;
            portSetpoint = 5;
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            starSetpoint = 0;
            portSetpoint = 0;

            // Configure/initialize servo
            if (!sonarServo.attached())
            {
                sonarServo.attach(SERVO1_PIN);
            }
            sonarServo.write(0);

            // Ready
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // Process
            while (occupationNotReady)
            {
                // Sweep control
                if (sweepUp)
                {
                    // We're sweeping up
                    lastSweep += 1;
                    if (lastSweep >= occupationMapDensity)
                    {
                        lastSweep -= 2;  // Go back one space
                        sweepUp = false; // Go back down!
                    }
                }
                else
                {
                    lastSweep -= 1;
                    if (lastSweep == 0)
                    {
                        sweepUp = true; // Go back up!
                    }
                }

                // Perform quick scan
                setBlue();                                                             // Set LED to blue
                sonarServo.write(lastSweep * (180.0 / occupationMapDensity));          // Begin moving
                vTaskDelay(110 / portTICK_PERIOD_MS);                                  // Wait for movement to finish
                occupationMap[lastSweep] = sonar.dist();                               // Record the distance
                Log.infoln("Distance at %d: %F", lastSweep, occupationMap[lastSweep]); // Log the distance recorded
                setRed();                                                              // Set the LED to red

                /**
                 * Compute the occupation map
                 */
                uint8_t _centroid = 0;

                // Iterate the map
                for (size_t i = 0; i < occupationMapDensity; i++)
                {
                    if (occupationMap[i] < occupationMap[_centroid] && occupationMap[i] != 0)
                    {
                        _centroid = i;
                    }
                }

                Log.infoln("Centroid of map occupation is at %d, distance %F", _centroid, occupationMap[_centroid]);

                if (occupationMap[occupationMapDensity - 1] != 0)
                {
                    occupationNotReady = false; // This bool lets the loop know we're ready.
                }

                // Detach (free timer2 for PWM)
            }

            // Done
            sonarServo.write(255 / 2);                             // Return to center
            Log.infoln("%s: Done scanning.", pcTaskGetName(NULL)); // Log completion
            setGreen();                                            // Set LED to green
            sonarServo.detach();

            // Do some driving!
            starSetpoint = 9;
            portSetpoint = 4;
            vTaskDelay(800 / portTICK_PERIOD_MS);
            starSetpoint = 9;
            portSetpoint = 9;
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            starSetpoint = 0;
            portSetpoint = 0;
            Log.infoln("%s: Done moving.", pcTaskGetName(NULL));
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
