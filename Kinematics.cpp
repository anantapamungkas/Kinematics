#include "Kinematics.h"

Kinematics::Kinematics(float integralMax)
  : Kp(1.0), Ki(0.0), Kd(0.0), integralMax(integralMax), integralTerm(0), previousError(0),
    derivativeFiltered(0), alpha(0.1), currentX(0), currentY(0), currentAngle(0),
    targetX(0), targetY(0), targetAngle(0), distanceError(0), angleError(0),
    maxSpeed(1.0), minSpeed(0.0) {  // Default values for speed
  
  // Default angles for standard mecanum drive
  angles[0] = 45;
  angles[1] = 135;
  angles[2] = 225;
  angles[3] = 315;
}

void Kinematics::setSpeedLimits(float max, float min) {
  maxSpeed = max;
  minSpeed = min;
}

void Kinematics::setPIDGains(float newKp, float newKi, float newKd) {
  Kp = newKp;
  Ki = newKi;
  Kd = newKd;
}

void Kinematics::setWheelAngles(float angleA, float angleB, float angleC, float angleD) {
  angles[0] = angleA;
  angles[1] = angleB;
  angles[2] = angleC;
  angles[3] = angleD;
}

void Kinematics::setSpeedLimits(float max, float min) {
  maxSpeed = max;
  minSpeed = min;
}

void Kinematics::updatePosition(float x, float y, float angle) {
  currentX = x;
  currentY = y;
  currentAngle = angle;
}

float Kinematics::calculatePID(float desiredValue, float currentValue) {
  float error = desiredValue - currentValue;

  // Apply deadband to avoid small unnecessary movements
  if (abs(error) < 0.5) return 0;

  // Integral term with anti-windup
  integralTerm += Ki * error;
  integralTerm = constrain(integralTerm, -integralMax, integralMax);

  // Smoothed derivative term
  derivativeFiltered = alpha * (error - previousError) + (1 - alpha) * derivativeFiltered;

  // Compute output
  float controlOutput = Kp * error + integralTerm + Kd * derivativeFiltered;
  previousError = error;

  // Constrain output speed
  return constrain(controlOutput, -maxSpeed, maxSpeed);
}

void Kinematics::inverse(float Vx, float Vy, float Vw) {
  float* wheels[] = { &wheelA, &wheelB, &wheelC, &wheelD };

  for (int i = 0; i < 4; i++) {
    float rad = angles[i] * PI / 180.0;
    *wheels[i] = (-sin(rad) * Vx + cos(rad) * Vy + Vw);
    *wheels[i] = constrain(*wheels[i], -maxSpeed, maxSpeed);
  }
}

// ✅ **New function to update errors**
void Kinematics::updateErrors() {
  float errorX = targetX - currentX;
  float errorY = targetY - currentY;
  distanceError = sqrt(errorX * errorX + errorY * errorY);  // Euclidean distance

  float error = targetAngle - currentAngle;
  angleError = fmod((error + 180), 360) - 180;  // Normalize to [-180, 180] range
}

void Kinematics::move(float newTargetX, float newTargetY, float newTargetAngle) {
  // Store the target values inside the class
  targetX = newTargetX;
  targetY = newTargetY;
  targetAngle = newTargetAngle;

  // Update error values
  updateErrors();

  // PID control
  float driveX = calculatePID(targetX, currentX);
  float driveY = calculatePID(targetY, currentY);
  float driveAngle = calculatePID(targetAngle, currentAngle);
  inverse(driveX, driveY, driveAngle);
}

void Kinematics::rotate(float newTargetAngle) {
  targetAngle = newTargetAngle;

  // Update angle error
  updateErrors();

  float drive = calculatePID(targetAngle, currentAngle);
  inverse(0, 0, drive);
}

void Kinematics::stop() {
  wheelA = 0;
  wheelB = 0;
  wheelC = 0;
  wheelD = 0;
}
