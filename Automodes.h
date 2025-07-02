#ifndef AUTOMODES_H
#define AUTOMODES_H

#include "TrapezoidalMotionProfile.h"

#define TOP_SPEED 0.6
#define TOP_ACCEL 1.5

static MotionProfile startWP = {
  .x = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTimeMs = -1
};

static MotionProfile approachWP = {
  .x = { .target = 0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTimeMs = -1
};

static MotionProfile lineWP = {
  .x = { .target = 0.700, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = -1,
  .pivot = -1,
  .intake = -1,
  .minimumTimeMs = -1
};

#endif
