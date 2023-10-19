// Annoying shortcuts

#include <Adafruit_NeoPixel.h>
#include "boardPins.h"

Adafruit_NeoPixel rgb(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

void setGreen()
{
    rgb.setPixelColor(0, 0, 1 * maxBrigh, 0); // Green
    rgb.show();                               // Push
}

void setRed()
{
    rgb.setPixelColor(0, 1 * maxBrigh, 0, 0); // Red
    rgb.show();                               // Push
}
