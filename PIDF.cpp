#include "PIDF.h"

PIDF::PIDF(float kp, float ki, float kd, float kf,
         float min_output, float max_output) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kd = kf;
    this->min_output = min_output;
    this->max_output = max_output;

    this->prev_error = 0;
    this->integral = 0;

    this->start_time = 0;
    this->prev_time = 0;

}

float PIDF::update(float error, float velocity) {
    unsigned long current_time = millis();
    float timestep;

    if (this->prev_time == 0) {
        this->start_time = current_time;
        timestep = 0.001; // Set an initial timestep of 1ms
    } else {
        timestep = (current_time - this->prev_time) / 1000.0; // Convert ms to seconds
    }

    this->prev_time = current_time;

    // Derivative
    float derivative = 0;
    if (timestep != 0) derivative = (error - this->prev_error) / timestep;

    // Integral
    this->integral += error * timestep;

    // Optional: clamp integral to avoid windup
    float max_integral = 1000;  // adjust as needed
    if (this->integral > max_integral) this->integral = max_integral;
    if (this->integral < -max_integral) this->integral = -max_integral;

    // Calculate output
    float output = this->kp * error + this->ki * this->integral + this->kd * derivative + this->kf * velocity;
    this->prev_error = error;

    // Bound output by min and max values
    output = constrain(output, this->min_output, this->max_output);


    //Serial.printf("kd %.3f  |  dt(S) %.5f  |  error(m) %.3f  |  derivative(m/s) %.3f  \n",this->kd,timestep,error,derivative);

    return output;
}

void PIDF::clear_history() {
    this->prev_error = 0;
    this->prev_time = 0;
}