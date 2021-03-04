#ifndef PYRO_H
#define PYRO_H

#include <Arduino.h>

class Pyro
{
public:
    Pyro(uint8_t pyroPin1, uint8_t pyroPin2);

    void setupPins();
    void triggerPin(uint8_t _pyroPin);

private:
    uint8_t _pyroPin;

    uint8_t _pyroPin1;
    uint8_t _pyroPin2;
};

#endif