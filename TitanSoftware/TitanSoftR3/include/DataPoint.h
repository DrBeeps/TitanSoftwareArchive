#ifndef DATA_POINT_H
#define DATA_POINT_H

#include <Arduino.h>

#include <FlightState.h>

union DataPoint 
{
  struct 
  {
    unsigned long elapsedFlightTime;
    
    FlightState logState;

    float batteryVoltage;

    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float altitude;

    float oriAngleX;
    float oriAngleY;
    float oriAngleZ;

    uint8_t heading;

    float tvcYTorque;
    float tvcZTorque;
    float tvcYAngle;
    float tvcZAngle;
  } data;

  uint8_t serializedData[150]; // Approx size
} sentData;

#endif