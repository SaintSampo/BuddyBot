// OpticalFlow.h
#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <math.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"


// Function declarations
void OpticalFlow_begin();
void OpticalFlow_resetTracking();
void OpticalFlow_rotateTracking(float angle);
float OpticalFlow_getX();
float OpticalFlow_getY();
float OpticalFlow_getTheta();
float OpticalFlow_getWrappedTheta();

#endif // OPTICALFLOW_H