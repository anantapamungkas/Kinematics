#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <cmath>

class Kinematics {
private:
  float Kp, Ki, Kd;
  float integralTerm, previousError;
  float integralMax;
  float maxSpeed, minSpeed;
  float derivativeFiltered;
  float alpha;  // Smoothing factor for derivative
  float angles[4];  // Dynamic wheel angles

  float calculatePID(float desiredValue, float currentValue);

public:
  float wheelA, wheelB, wheelC, wheelD;
  float currentX, currentY, currentAngle;  // Updated dynamically from sensors

  Kinematics(float maxSpeed, float minSpeed = 0, float integralMax = 10.0);

  void setPIDGains(float Kp, float Ki, float Kd);
  void setWheelAngles(float angleA, float angleB, float angleC, float angleD);
  void updatePosition(float x, float y, float angle);
  void inverse(float Vx, float Vy, float Vw);
  void move(float targetX, float targetY, float targetAngle);
  void rotate(float targetAngle);
};

#endif
