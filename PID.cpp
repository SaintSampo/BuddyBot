#include "PID.h"

PID::PID(float kp, float ki, float kd, 
         float min_output, float max_output) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->min_output = min_output;
    this->max_output = max_output;

    this->prev_error = 0;

    this->start_time = 0;
    this->prev_time = 0;

}

float PID::update(float error) {
    unsigned long current_time = millis();
    float timestep;

    if (this->prev_time == 0) {
        this->start_time = current_time;
        timestep = 0.001; // Set an initial timestep of 1ms
    } else {
        timestep = (current_time - this->prev_time) / 1000.0; // Convert ms to seconds
    }

    this->prev_time = current_time;

    float derivative = 0;
    if(timestep != 0) derivative = (error - this->prev_error) / timestep;

    // Calculate output
    //float output = this->kp * error + this->ki * this->prev_integral + this->kd * derivative;
    float output = this->kp * error + this->kd * derivative;
    this->prev_error = error;

    // Bound output by min and max values
    output = constrain(output, this->min_output, this->max_output);


    //Serial.printf("kd %.3f  |  dt(S) %.5f  |  error(m) %.3f  |  derivative(m/s) %.3f  \n",this->kd,timestep,error,derivative);

    return output;
}

void PID::clear_history() {
    this->prev_error = 0;
    this->prev_time = 0;
}