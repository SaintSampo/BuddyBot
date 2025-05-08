#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    PID(float kp = 1.0, float ki = 0.0, float kd = 0.0, 
        float min_output = 0.0, float max_output = 1.0
        );

    float update(float error);
    void clear_history();

private:

    float kp, ki, kd;
    float min_output, max_output;

    float prev_error;
    unsigned long start_time, prev_time;
};

#endif