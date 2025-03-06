#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <cmath>

class Kinematics {
private:
  float Kp, Ki, Kd;
  float integralTerm, previousError;
  float integralMax;
  float maxSpeed, minSpeed;  // Now public and adjustable
  float derivativeFiltered;
  float alpha;  // Smoothing factor for derivative
  float angles[4];  // Dynamic wheel angles

  float calculatePID(float desiredValue, float currentValue);
  void updateErrors();  // Private function to update distanceError & angleError

public:
  float wheelA, wheelB, wheelC, wheelD;
  float currentX, currentY, currentAngle;  // Updated dynamically from sensors
  float targetX, targetY, targetAngle;     // Store target values

  // Publicly accessible error variables
  float distanceError;
  float angleError;

  Kinematics(float integralMax = 10.0);

  void setSpeedLimits(float max, float min);
  void setPIDGains(float Kp, float Ki, float Kd);
  void setWheelAngles(float angleA, float angleB, float angleC, float angleD);
  void updatePosition(float x, float y, float angle);
  void inverse(float Vx, float Vy, float Vw);
  void move(float targetX, float targetY, float targetAngle);
  void rotate(float targetAngle);
  void stop();
};

#endif
