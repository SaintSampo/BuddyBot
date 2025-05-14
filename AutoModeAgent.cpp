#include "AutoModeAgent.h"

extern NoU_Drivetrain drivetrain;

float X_Ku = 60.0;
float X_Tu = 1.0/(150.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_P = 0.6 * X_Ku;
float X_D = 0.08 * X_Ku * X_Tu;
float X_F = 1.25; // 1 effort ~= 0.8 m/s -> 1/0.8 = 1.25

PIDF xPID(X_P, 0, X_D, X_F, -1.0, 1.0);
PIDF yPID(X_P, 0, X_D, X_F, -1.0, 1.0);

float T_Ku = 2.8;
float T_Tu = 1.0/(130.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float T_P = 0.6 * T_Ku;
float T_D = 0.075 * T_Ku * T_Tu;
float T_F = 0.32; // 1 effort ~= pi rad/s -> 1/pi ~= 0.32

PIDF thetaPID(T_P, 0, T_D, T_F, -1.0, 1.0);

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

  Pose fieldStartPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  unsigned long startTime = millis();
  
  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  while(true) {
    Pose fieldCurrentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - startTime) / 1000.0;

    Pose fieldTargetPose = {xProfile.distance(elapsedTime), yProfile.distance(elapsedTime), thetaProfile.distance(elapsedTime)};

    Pose fieldErrorPose = (fieldStartPose + fieldTargetPose) - fieldCurrentPose;
    Pose robotErrorPose = fieldErrorPose.toRobotFrame(fieldCurrentPose.theta);

    Pose fieldTargetVelocity = {xProfile.velocity(elapsedTime), yProfile.velocity(elapsedTime), thetaProfile.velocity(elapsedTime)};
    Pose robotTargetVelocity = fieldTargetVelocity.toRobotFrame(fieldCurrentPose.theta);


    effortX = xPID.update(robotErrorPose.x, robotTargetVelocity.x);
    effortY = yPID.update(robotErrorPose.y, robotTargetVelocity.y);
    effortTheta = thetaPID.update(robotErrorPose.theta, robotTargetVelocity.theta);

    drivetrain.holonomicDrive(effortX, effortY, effortTheta);
    
    //Serial.printf("elapsedTime(S): %.3f  |  X (s): %.3f  |  Y (S): %.2f  |  theta(s): %.2f \n",elapsedTime,xProfile.totalTime(), yProfile.totalTime(), thetaProfile.totalTime());
    //Serial.printf("time (S): %.2f of %.2f  |  startX(m): %.3f  |  currentX(m): %.3f  |  targetX(m): %.3f  |  errorX(m): %.3f  |  effortX(m): %.3f \n",elapsedTime,xProfile.totalTime(),startPose.x,currentPose.x,targetPose.x,fieldErrorPose.x,effortX);
    //Serial.printf("time (S): %.2f of %.2f  |  start(rad): %.3f  |  current(rad): %.3f  |  target(rad): %.3f  |  effort(rad): %.3f \n",elapsedTime,thetaProfile.totalTime(),fieldStartPose.theta,fieldCurrentPose.theta,targetPose.theta,effortTheta);
    //Serial.printf("time (S): %.2f of %.2f  |  effortX(m): %.3f  |  effortY(m): %.3f  |  effortTheta(rad): %.3f \n",elapsedTime,thetaProfile.totalTime(),effortX,effortY,effortTheta);
    //Serial.printf("time (S): %.2f of %.2f  |  fieldErrorX(m): %.3f  |  fieldErrorY(m): %.3f  |  robotErrorX(m): %.3f  |  robotErrorY(m): %.3f \n",elapsedTime,thetaProfile.totalTime(),fieldErrorPose.x,fieldErrorPose.y,robotErrorPose.x,robotErrorPose.y);


    if(elapsedTime > xProfile.totalTime() + 0 && elapsedTime > yProfile.totalTime() + 0 && elapsedTime > thetaProfile.totalTime() + 0){
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void AutoModeAgent_holdStability(bool holdX, bool holdY, bool holdTheta) {
  Pose startPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};
  
  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  while (true) {
    Pose currentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

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

    //Serial.printf("x (m): %.3f  |  y (m): %.3f  |  theta (rad): %.2f \n",OpticalFlow_getX(),OpticalFlow_getY(), OpticalFlow_getTheta());
    Serial.printf("x (m): %.3f  |  y (m): %.3f  |  theta (rad): %.2f \n",fieldErrorPose.x,fieldErrorPose.y,fieldErrorPose.theta);
    //Serial.printf("startX(m) %.3f  |  currentX(m) %.3f  |  EfieldX(m) %.3f  |  ErobotX(m) %.3f  |  effort %.3f  |   \n",startPose.x,currentPose.x,fieldErrorPose.x,robotErrorPose.x,effortX);
    //Serial.printf("start(rad) %.3f  |  current(rad) %.3f  |  Efield(rad) %.3f  |  Erobot(rad) %.3f  |  effort %.3f  |   \n",startPose.theta,currentPose.theta,fieldErrorPose.theta,robotErrorPose.theta,effortTheta);

    delay(10);
  }
}
