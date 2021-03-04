#include "Rocket.h"

Rocket::Rocket(FlightState currentState)
{
    _currentState = currentState;
}

float Rocket::calculateThrust(float _mass, float _accel)
{
    return _mass * _accel;
}

void Rocket::startup()
{
    _currentState = GROUND_IDLE;
}

void Rocket::updateTiming(double dt, uint64_t thisLoopMicros)
{
    _dt = dt;
    _thisLoopMicros = thisLoopMicros;
    dt = ((double)(_thisLoopMicros - _flightStartMicros) / 1000000.);
    _flightStartMicros = _thisLoopMicros;
}
