#ifndef TRAPEZOIDAL_MOTION_PROFILE_H
#define TRAPEZOIDAL_MOTION_PROFILE_H

#include <Arduino.h>

// Define the MotionProfile structure
struct Profile {
  float target;
  float startRate;
  float endRate;
  float maxAbsoluteRate;
  float maxAccel;
  float minimumTime;
};

struct MotionProfile {
  Profile x;
  Profile y;
  Profile theta;
};

class TrapezoidalMotionProfile {
public:
    TrapezoidalMotionProfile(Profile p);

    float distance(float t);
    float velocity(float t);
    float totalTime();

private:
    int direction;
    float d, v0, vf, amax, vmax;
    float da, dd, dc;
    float vpeak;
    float ta, tc, td;
    float total_time;
};

#endif
