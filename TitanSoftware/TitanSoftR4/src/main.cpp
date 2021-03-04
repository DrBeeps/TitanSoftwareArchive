#include <Arduino.h>

#include "FlightState.h"
#include "DataPoint.h"

#include "Orientation.h"
#include "PID.h"

#include "TVCMount.h"
#include "Pyro.h"

// ====================
// CUSTOM CLASSES
// ====================

TVCMount tvcMount(37, 36); // Z SERVO -> 37 Y SERVO -> 36
Pyro pyro(10, 33);



// ====================
// IMU
// ====================

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();



// ====================
// TIME
// ====================

uint64_t thisLoopMicros;
uint64_t calGyroMicros;
uint64_t flightStartMicros;
uint64_t calGyroLastMicros;
uint64_t launchDetectStart;
uint64_t burnoutDetectStart;
uint64_t dataLogMicros;

double dt;
double dtPID;
double lastPIDUpdate;

uint32_t nextServoWrite;
const uint8_t servoHz = 16;
const uint32_t servoWriteSpacing = 1000000 / servoHz;



// ====================
// ORIENTATION + PID
// ====================

Orientation ori;
LSM9DS1RawData imuRawData;
EulerAngles gyroData;
EulerAngles gyroOut;

float kp = 0.3;
float ki = 0.2;
float kd = 0.15;
float setpoint = 0;
PID pidY = {kp, ki, kd, setpoint};
PID pidZ = {kp, ki, kd, setpoint};

LSM9DS1CalGyroData calGyro;

double pidYOut, pidZOut;
double torqueY, torqueZ;
double angleYOut, angleZOut;

// ====================
// STATE
// ====================
FlightState currentState;

const uint8_t launchDetect = 12;
const int8_t burnoutDetect = -11;

const uint64_t launchDetectTime = 100000;



// ====================
// MISC / PROTO FUNCTIONS
// ====================

// Calibrate Gyros

void beginTimingGyroCal()
{
  calGyroLastMicros = micros();
}

void updateTimingGyroCal()
{
  calGyroMicros = micros();
  dt = ((double)(calGyroMicros - calGyroLastMicros) / 1000000.);
  calGyroLastMicros = calGyroMicros;
}

// Torque PID



// ====================
// FUNCTIONS
// ====================

void startup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(1);
  }
  if (!lsm.begin())
  {
    Serial.println("LSM9DS1 could not be initialized, check your wiring!");
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  currentState = GROUND_IDLE;
}

void beginTiming()
{
  nextServoWrite = micros() + servoWriteSpacing;
  flightStartMicros = micros();
}

void updateTiming()
{
  thisLoopMicros = micros();
  dt = ((double)(thisLoopMicros - flightStartMicros) / 1000000.);
  flightStartMicros = thisLoopMicros;
}

void checkSensors()
{
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  imuRawData.gyroX = g.gyro.x;
  imuRawData.gyroY = g.gyro.y;
  imuRawData.gyroZ = g.gyro.z;
  gyroData.roll = imuRawData.gyroX - calGyro.gXOffset;
  gyroData.pitch = imuRawData.gyroY - calGyro.gXOffset;
  gyroData.yaw = imuRawData.gyroZ - calGyro.gXOffset;

  imuRawData.accelX = a.acceleration.x;
  imuRawData.accelY = a.acceleration.y;
  imuRawData.accelZ = a.acceleration.z;

  imuRawData.magX = m.magnetic.x;
  imuRawData.magY = m.magnetic.y;
  imuRawData.magZ = m.magnetic.z;
}

void updateOri()
{
  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();

  Serial.print("Y => \t"); Serial.print(gyroOut.pitch * RAD_TO_DEG); Serial.print("\t");
  Serial.print("Z => \t"); Serial.print(gyroOut.yaw * RAD_TO_DEG); Serial.print("\n");

  // if (gyroOut.pitch >= 30 || gyroOut.pitch <= -30 || gyroOut.yaw >= 30 || gyroOut.yaw <= -30)
  // {
  //   currentState = ABORT;
  // }
}

void calGyros()
{
  for (int i = 0; i <= 100; i++)
  {
    updateTimingGyroCal();
    checkSensors();
    calGyro.gXArr += gyroOut.roll;
    calGyro.gYArr += gyroOut.pitch;
    calGyro.gZArr += gyroOut.yaw;
  }
  calGyro.gXOffset = calGyro.gXArr / 100;
  calGyro.gYOffset = calGyro.gYArr / 100;
  calGyro.gZOffset = calGyro.gZArr / 100;

  Serial.print("gXOffset => \t"); Serial.print(calGyro.gXOffset); Serial.print("\t");
  Serial.print("gYOffset => \t"); Serial.print(calGyro.gYOffset); Serial.print("\t");
  Serial.print("gZOffset => \t"); Serial.print(calGyro.gZOffset); Serial.println();
}

void checkLaunch()
{
  if(imuRawData.accelX < launchDetect) launchDetectStart = thisLoopMicros;
  if(thisLoopMicros > launchDetectStart + launchDetectTime) currentState = POWERED_FLIGHT;
  Serial.print("Accel X "); Serial.print(imuRawData.accelX); Serial.print("\t");
}

float calculateThrust(float _mass, float _accel)
{
  return _mass * _accel;
}

/* 
  measuredForce = calculateThrust(MASS, imuRawData.accelX);

  tvcXPIDOutput = tvcXPID.update(rocketOrientationX);

  tvcX.update(asin(tvcXPIDOutput / (measuredForce / whateverTheRadiusOfYourMountIs)));
*/

void updatePID()
{
  dtPID = (double)(thisLoopMicros - lastPIDUpdate) / 1000000.;
  lastPIDUpdate = thisLoopMicros;

  pidYOut = pidY.update(gyroData.pitch * RAD_TO_DEG, dtPID);
  pidZOut = pidZ.update(gyroData.yaw * RAD_TO_DEG, dtPID);

  angleYOut = constrain(pidYOut, -5, 5);
  angleZOut = constrain(pidZOut, -5, 5);

  Serial.print("Y OUT => "); Serial.print(angleYOut); Serial.print("\t");
  Serial.print("Z OUT => "); Serial.print(angleZOut); Serial.print("\n");

  if (thisLoopMicros >= nextServoWrite)
  {
    tvcMount.setAngle(angleYOut, angleZOut);
    nextServoWrite += servoWriteSpacing;
  }
}

void checkApogee()
{

}
void activateChutes()
{

}

void checkLanded()
{

}

void showSafe()
{

}

void abort()
{

}



// ====================
// SETUP + LOOP
// ====================

void setup() 
{
  tvcMount.setupPins();
  tvcMount.setOffsets(5, 0);
  pyro.setupPins();

  startup();

  // beginTimingGyroCal();
  // calGyros();

  beginTiming();
}

void loop() 
{
  updateTiming();
  checkSensors();
  updateOri();
  updatePID();
  /*
  switch(currentState)
  {
    case GROUND_IDLE:
      checkLaunch();
      // currentState = POWERED_FLIGHT;
      break;
    case POWERED_FLIGHT:
      updatePID();
      // checkBurnout();
      break;
    case UNPOWERED_FLIGHT:
      checkApogee();
      break;
    case BALLISTIC_DESCENT:
      checkChuteStop();
      break;
    case CHUTE_DESCENT:
      checkLanded();
      break;
    case GROUND_SAFE:
      showSafe();
      break;
    case ABORT:
      abort();
      break;
  }
  */
}