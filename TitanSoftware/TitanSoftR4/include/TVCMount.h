#ifndef TVC_MOUNT_H
#define TVC_MOUNT_H

#include <Arduino.h>
#include <Servo.h>

class TVCMount
{
public:
    TVCMount(uint8_t zServoPin, uint8_t yServoPin);
    
    void setupPins();
    void setOffsets(int zServoOffset, int yServoOffset);
    void setAngle(uint8_t zAngle, uint8_t yAngle);
    void setServos(uint8_t zAngle, uint8_t yAngle);
    void home();

    float tvcMountForce;
private:
    uint8_t _servoGearRatio = 6;
    uint8_t _homeAngle = 90;

    Servo _zServo;
    uint8_t _zServoPin;
    int _zServoOffset;

    Servo _yServo;
    uint8_t _yServoPin;
    int _yServoOffset;
};

#endif