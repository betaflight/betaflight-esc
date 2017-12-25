#include "include.h"

#define CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static void Beep(uint16_t hz, uint16_t length, uint8_t volume)
{
    uint32_t off = 1000 / (hz / 1000);
    uint32_t count = (hz / 1000) * length;

    volume = CONSTRAIN(volume, 1, 15);

    motor_pwm_set_freewheeling();
    delayMs(2);

    AFetLoOn();

    for (uint16_t x = 0; x < count; x++)
    {
        AFetHiOn();
        delayUs(volume);
        AFetHiOff();
        delayUs(off);

        BFetHiOn();
        delayUs(6);
        BFetHiOff();
        delayUs(6);
    }

    motor_pwm_set_freewheeling();
}

void PlayC7(uint16_t ms)
{
    Beep(2093, ms, 10);
}

void PlayCs7(uint16_t ms)
{
    Beep(2217, ms, 10);
}

void PlayD7(uint16_t ms)
{
    Beep(2349, ms, 10);
}

void PlayDs7(uint16_t ms)
{
    Beep(2489, ms, 10);
}

void PlayE7(uint16_t ms)
{
    Beep(2637, ms, 10);
}

void PlayF7(uint16_t ms)
{
    Beep(2793, ms, 10);
}

void PlayFs7(uint16_t ms)
{
    Beep(2959, ms, 10);
}

void PlayG7(uint16_t ms)
{
    Beep(3135, ms, 10);
}

void PlayGs7(uint16_t ms)
{
    Beep(3322, ms, 10);
}

void PlayA7(uint16_t ms)
{
    Beep(2093, ms, 10);
}

void PlayAs7(uint16_t ms)
{
    Beep(2093, ms, 10);
}

void PlayB7(uint16_t ms)
{
    Beep(2093, ms, 10);
}
