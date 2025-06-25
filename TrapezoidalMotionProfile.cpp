#include "TrapezoidalMotionProfile.h"

TrapezoidalMotionProfile::TrapezoidalMotionProfile(Profile p) {
    direction = (p.target >= 0) ? 1 : -1;

    d = fabs(p.target);
    v0 = p.startRate * direction;
    vf = p.endRate * direction;
    vmax = fabs(p.maxAbsoluteRate);
    amax = fabs(p.maxAccel);

    da = (vmax * vmax - v0 * v0) / (2.0f * amax);
    dd = (vmax * vmax - vf * vf) / (2.0f * amax);
    //Serial.printf("d (m): %.3f  | da (m): %.3f  |  dd(m): %.3f \n",d,da,dd);

    if (da + dd > d) {
        vpeak = sqrtf(max((2.0f * amax * d + v0 * v0 + vf * vf) / 2.0f, 0.0f));
        if (vpeak > vmax) vpeak = vmax;
        da = (vpeak * vpeak - v0 * v0) / (2.0f * amax);
        dd = (vpeak * vpeak - vf * vf) / (2.0f * amax);
        dc = 0.0f;
    } else {
        vpeak = vmax;
        dc = d - da - dd;
    }

    ta = (vpeak - v0) / amax;
    tc = (dc > 0.0f) ? dc / vpeak : 0.0f;
    td = (vpeak - vf) / amax;
    //Serial.printf("a (s): %.3f  |  c (S): %.3f  |  d (s): %.3f \n",ta,tc,td);
    total_time = ta + tc + td;
}

float TrapezoidalMotionProfile::distance(float t) {
    float dist = 0.0f;

    if (t < 0.0f) {
        dist = 0.0f;
    } else if (t < ta) {
        // Acceleration phase: integrate velocity from 0 to t
        dist = v0 * t + 0.5f * amax * t * t;
    } else if (t < ta + tc) {
        // Constant velocity phase
        float ta_dist = v0 * ta + 0.5f * amax * ta * ta;
        float dt = t - ta;
        dist = ta_dist + vpeak * dt;
    } else if (t < ta + tc + td) {
        // Deceleration phase
        float ta_dist = v0 * ta + 0.5f * amax * ta * ta;
        float tc_dist = vpeak * tc;
        float dt = t - ta - tc;
        dist = ta_dist + tc_dist + vpeak * dt - 0.5f * amax * dt * dt;
    } else {
        // After motion is complete
        dist = d;
    }

    return direction * dist;
}

float TrapezoidalMotionProfile::velocity(float t) {
    float vel = 0.0f;

    if (t < 0.0f) {
        vel = 0.0f;
    } else if (t < ta) {
        // Acceleration phase
        vel = v0 + amax * t;
    } else if (t < ta + tc) {
        // Constant velocity phase
        vel = vpeak;
    } else if (t < ta + tc + td) {
        // Deceleration phase
        float dt = t - ta - tc;
        vel = vpeak - amax * dt;
    } else {
        // After motion is complete
        vel = 0.0f;
    }

    return direction * vel;
}

float TrapezoidalMotionProfile::totalTime() {
    return total_time;
}
