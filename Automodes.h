#ifndef AUTOMODES_H
#define AUTOMODES_H

#include "TrapezoidalMotionProfile.h"

#define TOP_SPEED 0.25 //absolute max speed 0.85 m/s
#define TOP_ACCEL 0.3

static MotionProfile reef1WP = {
  .x = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 0,
  .pivot = 0,
  .intake = 0,
  .minimumTime = 2.0
};

static MotionProfile midwayWP = {
  .x = { .target = 0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 0,
  .pivot = 0,
  .intake = 0,
  .minimumTime = 2.0
};

static MotionProfile lineWP = {
  .x = { .target = 0.550, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 0,
  .pivot = 0,
  .intake = 0,
  .minimumTime = 2.0
};

static MotionProfile approach1HighWP = {
  .x = { .target = 0.0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.010, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 1400,
  .pivot = 550,
  .intake = -0.8,
  .minimumTime = 0.5
};

static MotionProfile score1HighWP = {
  .x = { .target = 0.0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.010, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 1450,
  .pivot = 650,
  .intake = 1,
  .minimumTime = 0.5
};

static MotionProfile loadingWP = {
  .x = { .target = -0.300, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.480, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = -6.0*DEG_TO_RAD, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 585,
  .pivot = 145,
  .intake = -0.8,
  .minimumTime = 4.0
};

static MotionProfile reef2WP = {
  .x = { .target = -0.060, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 585,
  .pivot = 145,
  .intake = -0.8,
  .minimumTime = 2.0
};

static MotionProfile approach2HighWP = {
  .x = { .target = -0.060, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.020, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 1400,
  .pivot = 550,
  .intake = -0.8,
  .minimumTime = 0.5
};

static MotionProfile score2HighWP = {
  .x = { .target = -0.060, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = -0.020, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 1450,
  .pivot = 650,
  .intake = 1,
  .minimumTime = 0.5
};

static MotionProfile endWP = {
  .x = { .target = -0.060, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .y = { .target = 0.0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = TOP_SPEED, .maxAccel = TOP_ACCEL},
  .theta = { .target = 0, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14},
  .elevator = 0,
  .pivot = 230,
  .intake = 0,
  .minimumTime = 0.1
};

#endif
