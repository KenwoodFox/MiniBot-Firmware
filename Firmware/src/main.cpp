#define MotorPWM_A 46 // left motor
#define MotorPWM_B 44 // right motor
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

#include <Arduino.h>

static volatile int count_A = 0;
static volatile int count_B = 0;

#define ENCODER 2
static volatile int16_t count = 0;
float RPM_A = 0;
float RPM_B = 0;

int PWM = 0;

// Prototype Functions
void ISR_A();
void ISR_B();

void setup()
{

    Serial.begin(9600);
    pinMode(MotorPWM_A, OUTPUT);
    pinMode(MotorPWM_B, OUTPUT);
    pinMode(INA1A, OUTPUT);
    pinMode(INA2A, OUTPUT);
    pinMode(INA1B, OUTPUT);
    pinMode(INA2B, OUTPUT);

    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), ISR_A, FALLING);
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(3), ISR_B, FALLING);
}

void RPM()
{
    count_A = 0;
    count_B = 0;
    delay(100);
    RPM_A = count_A * 3.125;
    RPM_B = count_B * 3.125;
    // Serial.print("RPM A = ");
    Serial.print(RPM_A);
    // Serial.print("RPM B = ");
    Serial.print(" ");
    Serial.println(RPM_B);
}

void ISR_A()
{
    count_A++;
}

void ISR_B()
{
    count_B++;
}

void Forward()
{
    analogWrite(MotorPWM_A, PWM);
    analogWrite(MotorPWM_B, PWM);
    digitalWrite(INA1A, 1);
    digitalWrite(INA2A, 0);
    digitalWrite(INA1B, 1);
    digitalWrite(INA2B, 0);
}

void Backward()
{
    analogWrite(MotorPWM_A, 255);
    analogWrite(MotorPWM_B, 255);
    digitalWrite(INA1A, 0);
    digitalWrite(INA2A, 1);
    digitalWrite(INA1B, 0);
    digitalWrite(INA2B, 1);
}

void Stop()
{
    analogWrite(MotorPWM_A, 0);
    analogWrite(MotorPWM_B, 0);
}

void loop()
{

    for (int i = 0; i < 52; i++)
    {
        delay(1000);
        PWM = i * 5;
        Forward();
        RPM();
    }
    Stop();
    while (1)
        ;
}
