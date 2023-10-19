/**
 * @file encoder.h
 * @brief Encoder methods and storage
 *
 */

volatile long int starPulse = 0;
volatile long int portPulse = 0;

void isrHandlerPort()
{
    // There is no way to reverse! Single phase encoders ONLY count up!
    portPulse += 1;
}

void isrHandlerStar()
{
    starPulse += 1;
}