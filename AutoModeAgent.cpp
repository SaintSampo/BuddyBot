#include "AutoModeAgent.h"

float X_Ku = 60.0 * 0.2;
float X_Tu = 1.0/(150.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float X_P = 0.6 * X_Ku;
float X_I = 0.5 * X_Tu;
float X_D = 0.125 * X_Tu;
float X_F = 1.0/1.15; // 1 effort ~= 0.8 m/s

PIDF xPID(X_P, X_I, X_D, X_F, -1.0, 1.0);
PIDF yPID(X_P, X_I, X_D, X_F, -1.0, 1.0);

float T_Ku = 2.8 * 0.8;
float T_Tu = 1.0/(130.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute
float T_P = 0.6 * T_Ku;
float T_I = 0.5 * T_Tu;
float T_D = 0.125 * T_Tu;
float T_F = 0.32; // 1 effort ~= pi rad/s -> 1/pi ~= 0.32

PIDF thetaPID(T_P, T_I, T_D, T_F, -1.0, 1.0);

float P_Ku = 2.0; //0.18 when FF is used
float P_Tu = 1.0/(200.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float P_P = 0.6 * P_Ku;
float P_D = 0.075 * P_Tu;
float P_F = 0.16; // 1 effort ~= 2pi rad/s -> 1/2pi ~= 0.16

PIDF pivotPID(P_P, 0, 0, P_F, -1, 1);

float E_Ku = 0.11;
float E_Tu = 1.0/(250.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float E_P = 0.6 * E_Ku;
float E_D = 0.075 * E_Tu;
float E_F = 0; // 1 effort ~= 2pi rad/s -> 1/2pi ~= 0.16

PIDF elevatorPID(E_P, 0, E_D, E_F, -1, 1);

ActuatorControl* controlStatePtr;

void AutoModeAgent_beginControlTask(ActuatorControl* ptr){

  controlStatePtr = ptr;

  // Start the actuator control task pinned to core 1
  xTaskCreatePinnedToCore(
    AutoModeAgent_controlTask,
    "ActuatorControl",
    4096,
    NULL,
    2,
    NULL,
    1
  );
}

void AutoModeAgent_controlTask(void* pvParameters) {
  Serial.println("Actuator control task started");

  for (;;) {
    if (controlStatePtr->enableDrivetrain) drivetrain_set(controlStatePtr->targetPose, controlStatePtr->targetVelocity);
    if (controlStatePtr->enablePivot) pivot_set(controlStatePtr->pivotTarget);
    if (controlStatePtr->enableElevator) elevator_set(controlStatePtr->elevatorTarget);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

Pose lastFieldEndPose = {0,0,0};

void AutoModeAgent_begin(QueueHandle_t profileQueue) {
  if (profileQueue == NULL) {
    Serial.println("Invalid motion profile queue!");
    return;
  }

  Serial.println("Running motion profiles");

  controlStatePtr->enableDrivetrain = true;
  controlStatePtr->enablePivot = true;
  controlStatePtr->enableElevator = true;

  MotionProfile profile;
  while (xQueueReceive(profileQueue, &profile, 0) == pdPASS) {
    Serial.println("Executing profile ");
    
    AutoModeAgent_executeProfile(profile);
    lastFieldEndPose = {profile.x.target, profile.y.target, profile.theta.target};

    Serial.println("Motion profile completed");
  }

  controlStatePtr->enableDrivetrain = false;
  controlStatePtr->enablePivot = false;
  controlStatePtr->enableElevator = false;

  drivetrain.holonomicDrive(0, 0, 0);
  elevator.set(0);
  pivot.set(0);

  Serial.println("All motion profiles completed");
}


void AutoModeAgent_executeProfile(MotionProfile p) {

  p.x.target -= lastFieldEndPose.x;
  p.y.target -= lastFieldEndPose.y;
  p.theta.target -= lastFieldEndPose.theta;

  TrapezoidalMotionProfile xProfile(p.x);
  TrapezoidalMotionProfile yProfile(p.y);
  TrapezoidalMotionProfile thetaProfile(p.theta);

  unsigned long startTime = millis();
  float elapsedTime = 0;

  float maxTime = 0;
  maxTime = max(maxTime, xProfile.totalTime());
  Serial.printf("maxTime(S): %.3f \n", maxTime);
  maxTime = max(maxTime, yProfile.totalTime());
  Serial.printf("maxTime(S): %.3f \n", maxTime);
  maxTime = max(maxTime, thetaProfile.totalTime());
  Serial.printf("maxTime(S): %.3f \n", maxTime);
  maxTime = max(maxTime, p.minimumTimeMs);

  Serial.printf("maxTime(S): %.3f \n", maxTime);
  while(elapsedTime <= maxTime) {
    unsigned long currentTime = millis();
    elapsedTime = (currentTime - startTime) / 1000.0;

    Pose fieldTargetPose = {xProfile.distance(elapsedTime), yProfile.distance(elapsedTime), thetaProfile.distance(elapsedTime)};
    Pose fieldTargetVelocity = {xProfile.velocity(elapsedTime), yProfile.velocity(elapsedTime), thetaProfile.velocity(elapsedTime)};

    controlStatePtr->targetPose = fieldTargetPose;
    controlStatePtr->targetVelocity = fieldTargetVelocity;
    
    //Serial.printf("elapsedTime(S): %.3f  |  X (s): %.3f  |  Y (S): %.2f  |  theta(s): %.2f \n",elapsedTime,xProfile.totalTime(), yProfile.totalTime(), thetaProfile.totalTime());
    //Serial.printf("time (S): %.2f of %.2f  |  currentX(m): %.3f \n",elapsedTime,xProfile.totalTime(),fieldTargetPose.x);
    //Serial.printf("time (S): %.2f of %.2f  |  start(rad): %.3f  |  current(rad): %.3f  |  target(rad): %.3f  |  effort(rad): %.3f \n",elapsedTime,thetaProfile.totalTime(),fieldStartPose.theta,fieldCurrentPose.theta,targetPose.theta,effortTheta);
    //Serial.printf("time (S): %.2f of %.2f  |  effortX(m): %.3f  |  effortY(m): %.3f  |  effortTheta(rad): %.3f \n",elapsedTime,thetaProfile.totalTime(),effortX,effortY,effortTheta);
    //Serial.printf("time (S): %.2f of %.2f  |  fieldErrorX(m): %.3f  |  fieldErrorY(m): %.3f  |  robotErrorX(m): %.3f  |  robotErrorY(m): %.3f \n",elapsedTime,thetaProfile.totalTime(),fieldErrorPose.x,fieldErrorPose.y,robotErrorPose.x,robotErrorPose.y);
  }
}

void drivetrain_set(Pose fieldJournyPose, Pose fieldTargetVelocity) {
  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  Pose fieldCurrentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  Pose fieldErrorPose = (lastFieldEndPose + fieldJournyPose) - fieldCurrentPose;
  Pose robotErrorPose = fieldErrorPose.toRobotFrame(fieldCurrentPose.theta);
  Pose robotTargetVelocity = fieldTargetVelocity.toRobotFrame(fieldCurrentPose.theta);

  effortX = xPID.update(robotErrorPose.x, robotTargetVelocity.x);
  effortY = yPID.update(robotErrorPose.y, robotTargetVelocity.y);
  effortTheta = thetaPID.update(robotErrorPose.theta, robotTargetVelocity.theta);

  drivetrain.holonomicDrive(effortX, effortY, effortTheta);

  Pose printPose = fieldCurrentPose;

  Serial.printf("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", printPose.x, printPose.y, printPose.theta);
}

void pivot_set(float targetAngle) {
  static float startAngle = pivot.getPosition();
  float effort = 0;

  float currentAngle = pivot.getPosition();
  float errorAngle =  (startAngle + targetAngle) - currentAngle;
  effort = pivotPID.update(errorAngle);

  pivot.set(effort);

  //Serial.printf("start(ticks): %.1f  |  current(ticks): %.3f  |  effort: %.1f \n",startAngle,currentAngle,effort);
}

void elevator_set(float targetDistance) {
  static float startDistance = elevator.getPosition();
  float effort = 0;

  float currentDistance = elevator.getPosition();
  float errorDistance =  (startDistance + targetDistance) - currentDistance;
  effort = elevatorPID.update(errorDistance);

  elevator.set(effort + 0.2);

  //Serial.printf("start(ticks): %.1f  |  current(ticks): %.3f  |  effort: %.1f \n",startDistance,currentDistance,effort);
}

// void Pivot_executeProfile(const Profile& p) {
  
//   TrapezoidalMotionProfile pivotProfile(p);

//   float startAngle = pivot.getPosition();

//   unsigned long startTime = millis();
  
//   float effortPivot = 0;

//   while(true) {
//     float currentAngle = pivot.getPosition();

//     unsigned long currentTime = millis();
//     float elapsedTime = (currentTime - startTime) / 1000.0;

//     float targetAngle = pivotProfile.distance(elapsedTime);
//     float errorAngle =  (startAngle + targetAngle) - currentAngle;
//     float targetVelocity = pivotProfile.velocity(elapsedTime);

//     effortPivot = pivotPID.update(errorAngle, targetVelocity);

//     pivot.set(effortPivot);

//     //Serial.printf("time (S): %.2f of %.2f, start(ticks): %.1f, current(ticks): %.3f, effort: %.1f \n",elapsedTime,pivotProfile.totalTime(),startAngle,currentAngle,effortPivot);
//     Serial.printf("current(ticks):%.1f,target(ticks):%.1f\n",currentAngle,targetAngle);


//     if(elapsedTime > pivotProfile.totalTime()){
//       break;
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
//   pivot.set(0);
// }
