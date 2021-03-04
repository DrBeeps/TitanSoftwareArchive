#ifndef ROCKET_H
#define ROCKET_H

#include <Arduino.h>

#include "FlightState.h"
#include "DataPoint.h"

#include "Orientation.h"
#include "PID.h"

#include "TVCMount.h"
#include "Pyro.h"

class Rocket
{
private:
    FlightState _currentState;

    // Timing
    double _dt;
    int _launchDetect = 12;
    int _burnoutDetect = 11;
    
    uint64_t _thisLoopMicros;
    uint64_t _launchDetectTime = 100000;

    const uint8_t _servoHz = 16;
    const uint32_t _servoWriteSpacing = 1000000 / _servoHz;


public:
    Rocket(FlightState currentState); // maybe also include a motor as an constructor arg

    float calculateThrust(float _mass, float _accel);

    void startup();
    void beginTiming();
    void updateTiming(double dt);
    void checkSensors();
    void checkLaunch();
    void updatePID();
    void checkApogee();
    void activateChutes();
    void checkLanded();
    void showSafe();
    void abort();

private:

// State Change Vars (to be placed in main.cpp)
/*



*/
};

#endif