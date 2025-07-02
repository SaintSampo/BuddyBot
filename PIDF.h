#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
public:
    PIDF(float kp = 1.0, float ki = 0.0, float kd = 0.0, float kf = 0.0,
        float min_output = 0.0, float max_output = 1.0
        );

    float update(float error, float velocity = 0);
    void clear_history();

private:

    float kp, ki, kd, kf;
    float min_output, max_output;

    float prev_error, integral;
    unsigned long start_time, prev_time;
};

#endif