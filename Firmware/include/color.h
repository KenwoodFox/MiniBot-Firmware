#include <Arduino.h>

const uint8_t numColors = 3;

enum Color
{
    RED,
    GREEN,
    BLUE,
};

inline Color operator++(Color &eDOW, int)
{
    const Color ePrev = eDOW;
    const int i = static_cast<int>(eDOW);
    eDOW = static_cast<Color>((i + 1) % numColors);
    return ePrev;
}