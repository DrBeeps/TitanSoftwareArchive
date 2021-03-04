#include <Pyro.h>

Pyro::Pyro(uint8_t pyroPin1, uint8_t pyroPin2)
{
    _pyroPin1 = pyroPin1;
    _pyroPin2 = pyroPin2;
}

void Pyro::setupPins()
{
    pinMode(_pyroPin1, OUTPUT);
    pinMode(_pyroPin2, OUTPUT);
}

void Pyro::triggerPin(uint8_t _pyroPin)
{
    digitalWrite(_pyroPin, HIGH);
    delay(500);
    digitalWrite(_pyroPin, LOW);
}
