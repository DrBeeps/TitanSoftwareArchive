#include <TVCMount.h>

TVCMount::TVCMount(uint8_t zServoPin, uint8_t yServoPin)
{
    _zServoPin = zServoPin;
    _yServoPin = yServoPin;
}

void TVCMount::setupPins()
{
    _zServo.attach(_zServoPin);
    _yServo.attach(_yServoPin);
}

void TVCMount::setOffsets(int zServoOffset, int yServoOffset)
{
    _zServoOffset = zServoOffset;
    _yServoOffset = yServoOffset;
}

void TVCMount::setAngle(uint8_t zAngle, uint8_t yAngle)
{
    _zServo.write(((zAngle + _zServoOffset) + _homeAngle) * _servoGearRatio);
    _yServo.write(((yAngle + _yServoOffset) + _homeAngle) * _servoGearRatio);
}

void TVCMount::setServos(uint8_t zAngle, uint8_t yAngle)
{
    _zServo.write((zAngle + _zServoOffset) + _homeAngle);
    _yServo.write((yAngle + _yServoOffset) + _homeAngle);
}

void TVCMount::home()
{
    setAngle(0, 0);
}