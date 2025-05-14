#ifndef AUTOMODEAGENT_H
#define AUTOMODEAGENT_H

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include "OpticalFlowAgent.h"
#include "TrapezoidalMotionProfile.h"
#include "DrivetrainHack.h"
#include "PIDF.h"

struct Pose {
  float x, y, theta;  // Position (x, y) and orientation (theta)

  // Add another Pose to this Pose
  Pose operator+(const Pose& other) const {
    Pose result;
    result.x = x + other.x;
    result.y = y + other.y;
    result.theta = theta + other.theta;
    return result;
  }

  // Subtract another Pose from this Pose
  Pose operator-(const Pose& other) const {
    Pose result;
    result.x = x - other.x;
    result.y = y - other.y;
    result.theta = theta - other.theta;
    return result;
  }

  // Scale the x and y by a scalar (theta unchanged)
  Pose operator*(float scale) const {
    return {x * scale, y * scale, theta};
  }

  // Allow float * Pose as well
  friend Pose operator*(float scale, const Pose& pose) {
    return pose * scale;  // Reuse member operator*
  }

  // Convert this pose from world frame to robot (holonomic) frame
  Pose toRobotFrame(float currentTheta) const {
    float cosTheta = cos(currentTheta);
    float sinTheta = sin(currentTheta);

    float robotX = cosTheta * x + sinTheta * y;
    float robotY = - sinTheta * x + cosTheta * y;

    return {robotX, robotY, theta};  // Orientation is typically not transformed here
  }
};

void AutoModeAgent_begin();
void AutoModeAgent_motionProfileTask(void *pvParameters);
void AutoModeAgent_executeProfile(const MotionProfile &p);
void AutoModeAgent_holdStability(bool holdX = true, bool holdY = true, bool holdAlpha = true);

#endif