#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>
// -----------------------------------------------------------------------------
// Units. Here because the sensitivities below are what they convert.
// -----------------------------------------------------------------------------
static constexpr float kGravity    = 9.80665f;
static constexpr float kMg2Ms2     = kGravity / 1000.0f;             // mg -> m/s^2
static constexpr float kMdps2Rads  = 1.0e-3f * (float)M_PI / 180.0f; // mdps -> rad/s

#endif  // CONSTANTS_H