#ifndef AUTOMODES_H
#define AUTOMODES_H

#include "TrapezoidalMotionProfile.h"

const MotionProfile leftTwoPiece = {
  .x = { .target = 0.200, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 0.080, .maxAccel = 3.00, .minimumTime = 0},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 0.300, .maxAccel = 1.500, .minimumTime = 0},
  .theta = { .target = 2 * 3.14, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14, .minimumTime = 0}
};

#endif
