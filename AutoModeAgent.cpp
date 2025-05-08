#include "AutoModeAgent.h"

extern NoU_Drivetrain drivetrain;

float X_Ku = 60.0;
float X_Tu = 1.0/(150.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_P = 0.6 * X_Ku;
float X_D = 0.08 * X_Ku * X_Tu;

PID xPID(X_P, 0, X_D, -1.0, 1.0);
PID yPID(X_P, 0, X_D, -1.0, 1.0);

float T_Ku = 2.8;
float T_Tu = 1.0/(130.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float T_P = 0.6 * T_Ku;
float T_D = 0.075 * T_Ku * T_Tu;

PID thetaPID(T_P, 0, T_D, -1.0, 1.0);

// === Global Motion Profiles ===
MotionProfile motionProfiles[] = {
  {
    .x = { .target = 100, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 300, .maxAccel = 600 },
    .y = { .target =   0, .startRate = 0, .endRate = 0, .maxAbsoluteRate =   0, .maxAccel =   0 },
    .theta = { .target =   0, .startRate = 0, .endRate = 0, .maxAbsoluteRate =   0, .maxAccel =   0 }
  },
  {
    .x = { .target = 200, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 300, .maxAccel = 600 },
    .y = { .target =   0, .startRate = 0, .endRate = 0, .maxAbsoluteRate =   0, .maxAccel =   0 },
    .theta = { .target =  90, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 180, .maxAccel = 360 }
  }
};

const int numProfiles = sizeof(motionProfiles) / sizeof(motionProfiles[0]);

void AutoModeAgent_begin() {
  TaskHandle_t currentMotionTaskHandle = nullptr;

  for (int i = 0; i < numProfiles; ++i) {
    Serial.printf("Launching Motion Profile %d\n", i);

    xTaskCreatePinnedToCore(
      AutoModeAgent_motionProfileTask, "MotionProfile", 4096,
      &motionProfiles[i], 2, &currentMotionTaskHandle, 1);

    while (currentMotionTaskHandle != NULL && eTaskGetState(currentMotionTaskHandle) != eDeleted) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  Serial.println("All motion profiles complete.");
}

void AutoModeAgent_motionProfileTask(void* pvParameters) {
  MotionProfile* profile = static_cast<MotionProfile*>(pvParameters);

  Serial.printf("Motion Profile started\n");

  AutoModeAgent_executeProfile(*profile);  // Pass by reference

  Serial.printf("Motion Profile completed\n");

  vTaskDelete(NULL);
}


void AutoModeAgent_executeProfile(const MotionProfile& p) {
  
  TrapezoidalMotionProfile xProfile(p.x);
  TrapezoidalMotionProfile yProfile(p.y);
  TrapezoidalMotionProfile thetaProfile(p.theta);

  Pose startPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};
  //Pose startPose = transformSensorToRobotCenter(sensorStartPose);

  unsigned long startTime = millis();
  
  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  while(true) {
    Pose currentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};
    //Pose currentPose = transformSensorToRobotCenter(sensorCurrentPose);

    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - startTime) / 1000.0;

    Pose targetPose = {xProfile.distance(elapsedTime), yProfile.distance(elapsedTime), thetaProfile.distance(elapsedTime)};

    Pose fieldErrorPose = (currentPose - startPose) + targetPose;

    float cosTheta = cos(currentPose.theta);
    float sinTheta = sin(currentPose.theta);
    
    Pose robotErrorPose = {fieldErrorPose.x * cosTheta + fieldErrorPose.y * sinTheta,
                          -fieldErrorPose.x * sinTheta + fieldErrorPose.y * cosTheta,
                          -fieldErrorPose.theta
                          };

    effortX = xPID.update(robotErrorPose.x);
    effortY = yPID.update(robotErrorPose.y);
    effortTheta = thetaPID.update(robotErrorPose.theta);

    drivetrain.holonomicDrive(effortX, effortY, effortTheta);
    
    //Serial.printf("elapsedTime(S): %.3f  |  X (s): %.3f  |  Y (S): %.2f  |  theta(s): %.2f \n",elapsedTime,xProfile.totalTime(), yProfile.totalTime(), thetaProfile.totalTime());
    Serial.printf("time (S): %.2f of %.2f  |  startX(m): %.3f  |  currentX(m)(S): %.3f  |  targetX(m): %.3f  |  robotErrorX(m): %.3f \n",elapsedTime,xProfile.totalTime(),startPose.x,currentPose.x,targetPose.x,robotErrorPose.x);

    if(elapsedTime > xProfile.totalTime() && elapsedTime > yProfile.totalTime() && elapsedTime > thetaProfile.totalTime()){
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void AutoModeAgent_holdStability(bool holdX, bool holdY, bool holdTheta) {
  Pose startPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};
  //Pose startPose = transformSensorToRobotCenter(sensorStartPose);
  
  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  while (true) {
    Pose currentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};
    //Pose currentPose = transformSensorToRobotCenter(sensorCurrentPose);

    Pose fieldErrorPose =  startPose - currentPose;

    float cosTheta = cos(currentPose.theta);
    float sinTheta = sin(currentPose.theta);
    
    Pose robotErrorPose = {fieldErrorPose.x * cosTheta + fieldErrorPose.y * sinTheta,
                          -fieldErrorPose.x * sinTheta + fieldErrorPose.y * cosTheta,
                          -fieldErrorPose.theta
                          };


    if(holdX) effortX = xPID.update(robotErrorPose.x);
    if(holdY) effortY = yPID.update(robotErrorPose.y);
    if(holdTheta) effortTheta = thetaPID.update(robotErrorPose.theta);

    drivetrain.holonomicDrive(effortX, effortY, effortTheta);

    //Serial.printf("Theta (rad): %.3f Start (mm): %.2f End (mm): %.2f Field Error (mm): %.2f Robot Error (mm): %.2f Forward Effort: %.2f\n",fieldErrorPose.theta, startPose.x*1000, currentPose.x*1000, fieldErrorPose.x*1000, robotErrorPose.x*1000, effortX);
    //Serial.printf("fepx: %.3f  |  fepy (mm): %.2f  |  rep (mm): %.2f \n",fieldErrorPose.x*1000, fieldErrorPose.y*1000,robotErrorPose.x*1000);
    //Serial.printf("raw: %.3f  |  fept (rad): %.3f  |  rept (rad): %.2f  |  effort: %.2f \n",OpticalFlow_getTheta(),fieldErrorPose.theta, robotErrorPose.theta, effortTheta);
    //Serial.printf("x (m): %.3f  |  y (m): %.3f  |  theta (rad): %.2f \n",OpticalFlow_getX(),OpticalFlow_getY(), OpticalFlow_getTheta());
    Serial.printf("startX(m) %.3f  |  currentX(m) %.3f  |  EfieldX(m) %.3f  |  ErobotX(m) %.3f  |  effort %.3f  |   \n",startPose.x,currentPose.x,fieldErrorPose.x,robotErrorPose.x,effortX);

    delay(10);
  }
}

Pose transformSensorToRobotCenter(Pose sensorPose) {
  //const float sensorOffsetX = 0.05; // meters
  const float sensorOffsetX = 0.00; //meters
  const float sensorOffsetY = 0.0;

  float cosTheta = cos(sensorPose.theta);
  float sinTheta = sin(sensorPose.theta);

  Pose robotCenterPose;
  robotCenterPose.x = sensorPose.x - sensorOffsetX * cosTheta + sensorOffsetY * sinTheta;
  robotCenterPose.y = sensorPose.y - sensorOffsetX * sinTheta - sensorOffsetY * cosTheta;
  robotCenterPose.theta = sensorPose.theta;
  return robotCenterPose;
}