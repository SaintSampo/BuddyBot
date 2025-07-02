#ifndef AUTOMODES_H
#define AUTOMODES_H

#include "TrapezoidalMotionProfile.h"

#define TOP_SPEED 0.3 //absolute max speed 0.85 m/s
#define TOP_ACCEL 1.5

static MotionProfile startWP = {
  .x = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTime = 2.0
};

static MotionProfile approachWP = {
  .x = { .target = 0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTime = 2.0
};

static MotionProfile lineWP = {
  .x = { .target = 0.200, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTime = 2.0
};

#endif
