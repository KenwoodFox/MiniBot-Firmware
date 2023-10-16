/**
 * @brief Settings and configurable templates
 *
 */

// LED
const int maxBrigh = 20; // Joe's eyes hurt!

// Template Structs
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

    // Pointers to external values
    volatile long int *encPtr;
};
