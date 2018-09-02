#include <Wire.h>
#include "Balance.h"
#include <math.h>

int32_t gYZero;
double angle; // millidegrees
double angleRate; // degrees/s
double distanceLeft;
double speedLeft;
//int32_t driveLeft;
double distanceRight;
double speedRight;
//int32_t driveRight;
//int16_t motorLeftSpeed;
//int16_t motorRightSpeed;
int16_t motorSpeed;
double motorSpeed1;
double motorSpeed2;
bool isBalancingStatus;
bool balanceUpdateDelayedStatus;
int16_t countsLeft;
int16_t countsRight;

int32_t i=0;

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
}

// This function contains the core algorithm for balancing a
// Balboa 32U4 robot.
void balance()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  angle = angle * 999 / 1000;

  // This variable measures how close we are to our basic
  // balancing goal - being on a trajectory that would cause us
  // to rise up to the vertical position with zero speed left at
  // the top.  This is similar to the fallingAngleOffset used
  // for LED feedback and a calibration procedure discussed at
  // the end of Balancer.ino.
  //
  // It is in units of millidegrees, like the angle variable, and
  // you can think of it as an angular estimate of how far off we
  // are from being balanced.
//  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  // Combine risingAngleOffset with the distance and speed
  // variables, using the calibration constants defined in
  // Balance.h, to get our motor response.  Rather than becoming
  // the new motor speed setting, the response is an amount that
  // is added to the motor speeds, since a *change* in speed is
  // what causes the robot to tilt one way or the other.
  motorSpeed1 = (((double)(
    + ANGLE_RESPONSE * angle //* angle * (angle < 0 ? -1 : 1)
    + ANGLE_RATE_RESPONSE * angleRate
    + DISTANCE_RESPONSE * (distanceRight+distanceLeft)/2
    + SPEED_RESPONSE * (speedRight+speedLeft)/2
    ))*MOTOR_VOLTAGE_RATIO);
//    Serial.print(motorSpeed);
//    Serial.print("\t");

  motorSpeed=.1*motorSpeed2+.9*motorSpeed1;
  motorSpeed2=motorSpeed1;
  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  else if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }
  
  //if (i < 10) {
  //  i++;
  //} else {
    Serial.print(angle);
    Serial.print("\t");
    Serial.print(angleRate);
    Serial.print("\t");
    Serial.print((distanceRight+distanceLeft)/2);
    Serial.print("\t");
    Serial.print((speedLeft+speedRight)/2);
    Serial.print("\t");
    Serial.println(motorSpeed);
    
    //Serial.println(imu.g.y);
  //  i = 0;
  //}

  motors.setSpeeds(1.4*motorSpeed, motorSpeed);
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/s to rad/s.
  angleRate = ((double)(imu.g.y - gYZero))/1800; // 1000 * pi / 180

  angle += angleRate * ((double)UPDATE_TIME_MS) / 1000.0;
}

void integrateEncoders()
{
  countsLeft = encoders.getCountsAndResetLeft();
  speedLeft = ((double)(countsLeft))*ENCODER_RATIO*1000/UPDATE_TIME_MS;
  if (distanceLeft < 45 && distanceLeft > -45) {
    distanceLeft += speedLeft;
  }

  countsRight = encoders.getCountsAndResetRight();
  speedRight = ((double)(countsRight))*ENCODER_RATIO*1000/UPDATE_TIME_MS;
  if (distanceRight < 45 && distanceRight > -45) {
    distanceRight += speedRight;
  }
}

void balanceUpdateSensors()
{
  imu.read();
  integrateGyro();
  integrateEncoders();
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
//  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  balanceUpdateSensors();
//  balanceDoDriveTicks();

//  if (imu.a.x < 0)
//  {
//    lyingDown();
//    isBalancingStatus = false;
//  }
//  else
//  {
    balance();
//    isBalancingStatus = true;
//  }
}

