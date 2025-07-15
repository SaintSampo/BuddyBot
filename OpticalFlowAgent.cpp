#include "OpticalFlowAgent.h"

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

bool otosConnected = false;
void OpticalFlow_begin(){
    
    // Attempt to begin the sensor
    if (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        return;
    }

    otosConnected = true;

    Serial.println("OTOS connected!");
    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu();
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);

    // Reset the tracking algorithm - this resets the position to the origin,
    myOtos.resetTracking();

    // For example, if the sensor is +50mm in the X direction from the center of the robot, offset would be {0.05,0,0}
    sfe_otos_pose2d_t offsetPose = {0,0,0};

    myOtos.setOffset(offsetPose);
}

void OpticalFlow_resetTracking(){
    myOtos.resetTracking();
}

void OpticalFlow_rotateTracking(float angle){
    sfe_otos_pose2d_t offsetPose = {0,0,angle};
    myOtos.setOffset(offsetPose);
}

float OpticalFlow_getX(){
    if(!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.x;
}

float OpticalFlow_getY(){
    if(!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.y;
}

float OpticalFlow_getTheta() {
    if(!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);
    
    static float lastWrappedTheta = 0;
    static float unwrappedTheta = 0;

    float currentWrappedTheta = myPosition.h;
    float delta = currentWrappedTheta - lastWrappedTheta;

    // Handle wraparound by checking if the jump is greater than π
    if (delta > M_PI) {
        delta -= 2 * M_PI;
    } else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }

    unwrappedTheta += delta;
    lastWrappedTheta = currentWrappedTheta;

    return unwrappedTheta;
}


float OpticalFlow_getWrappedTheta(){
    if(!otosConnected) return 0.0;

    sfe_otos_pose2d_t myPosition;
    myOtos.getPosition(myPosition);

    return myPosition.h;
}

