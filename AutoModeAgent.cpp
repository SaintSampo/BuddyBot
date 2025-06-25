#include "AutoModeAgent.h"

extern NoU_Drivetrain drivetrain;
extern NoU_Motor pivot;

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

float P_Ku = 2.0; //0.18 when FF is used
float P_Tu = 1.0/(200.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float P_P = 0.6 * P_Ku;
float P_D = 0.075 * P_Ku * P_Tu;
float P_F = 0.16; // 1 effort ~= 2pi rad/s -> 1/2pi ~= 0.16

PIDF pivotPID(P_P, 0, 0, P_F, -1, 1);

float E_Ku = 0.11;
float E_Tu = 1.0/(250.0/60.0); //Period = 1/(BPM/60), BPM is measuring full cycles per minute

float E_P = 0.6 * E_Ku;
float E_D = 0.075 * E_Ku * E_Tu;
float E_F = 0; // 1 effort ~= 2pi rad/s -> 1/2pi ~= 0.16

PIDF elevatorPID(E_P, 0, E_D, E_F, -1, 1);

MotionProfile leftTwoPiece = {
  .x = { .target = 0.200, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 0.080, .maxAccel = 3.00, .minimumTime = 0},
  .y = { .target = 0.000, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 0.300, .maxAccel = 1.500, .minimumTime = 0},
  .theta = { .target = 2 * 3.14, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 1 * 3.14, .maxAccel = 4 * 3.14, .minimumTime = 0}
};

// const int numProfiles = sizeof(motionProfiles) / sizeof(motionProfiles[0]);

// void AutoModeAgent_begin() {
//   TaskHandle_t currentMotionTaskHandle = nullptr;

//   for (int i = 0; i < numProfiles; ++i) {
//     Serial.printf("Launching Motion Profile %d\n", i);

//     xTaskCreatePinnedToCore(
//       AutoModeAgent_motionProfileTask, "MotionProfile", 4096,
//       &motionProfiles[i], 2, &currentMotionTaskHandle, 1);

//     while (currentMotionTaskHandle != NULL && eTaskGetState(currentMotionTaskHandle) != eDeleted) {
//       vTaskDelay(pdMS_TO_TICKS(100));
//     }
//   }

//   Serial.println("All motion profiles complete.");
// }

// void AutoModeAgent_motionProfileTask(void* pvParameters) {
//   MotionProfile* profile = static_cast<MotionProfile*>(pvParameters);

//   Serial.printf("Motion Profile started\n");

//   AutoModeAgent_executeProfile(*profile);  // Pass by reference

//   Serial.printf("Motion Profile completed\n");

//   vTaskDelete(NULL);
// }


void AutoModeAgent_executeProfile(const MotionProfile& p) {
  
  TrapezoidalMotionProfile xProfile(p.x);
  TrapezoidalMotionProfile yProfile(p.y);
  TrapezoidalMotionProfile thetaProfile(p.theta);

  unsigned long startTime = millis();

  while(true) {
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - startTime) / 1000.0;

    Pose fieldTargetPose = {xProfile.distance(elapsedTime), yProfile.distance(elapsedTime), thetaProfile.distance(elapsedTime)};
    Pose fieldTargetVelocity = {xProfile.velocity(elapsedTime), yProfile.velocity(elapsedTime), thetaProfile.velocity(elapsedTime)};

    drivetrain_set(fieldTargetPose, fieldTargetVelocity);
    
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


void drivetrain_set(Pose fieldTargetPose, Pose fieldTargetVelocity) {
  static Pose fieldStartPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  float effortX = 0;
  float effortY = 0;
  float effortTheta = 0;

  Pose fieldCurrentPose = {OpticalFlow_getX(), OpticalFlow_getY(), OpticalFlow_getTheta()};

  Pose fieldErrorPose = (fieldStartPose + fieldTargetPose) - fieldCurrentPose;
  Pose robotErrorPose = fieldErrorPose.toRobotFrame(fieldCurrentPose.theta);
  Pose robotTargetVelocity = fieldTargetVelocity.toRobotFrame(fieldCurrentPose.theta);

  effortX = xPID.update(robotErrorPose.x, robotTargetVelocity.x);
  effortY = yPID.update(robotErrorPose.y, robotTargetVelocity.y);
  effortTheta = thetaPID.update(robotErrorPose.theta, robotTargetVelocity.theta);

  drivetrain.holonomicDrive(effortX, effortY, effortTheta);

  Pose printPose = robotErrorPose;

  Serial.printf("X: %.3f  |  Y: %.3f  |  theta: %.3f \n", printPose.x, printPose.y, printPose.theta);
}


void Pivot_executeProfile(const Profile& p) {
  
  TrapezoidalMotionProfile pivotProfile(p);

  float startAngle = pivot.getPosition();

  unsigned long startTime = millis();
  
  float effortPivot = 0;

  while(true) {
    float currentAngle = pivot.getPosition();

    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - startTime) / 1000.0;

    float targetAngle = pivotProfile.distance(elapsedTime);
    float errorAngle =  (startAngle + targetAngle) - currentAngle;
    float targetVelocity = pivotProfile.velocity(elapsedTime);

    effortPivot = pivotPID.update(errorAngle, targetVelocity);

    pivot.set(effortPivot);

    //Serial.printf("time (S): %.2f of %.2f, start(ticks): %.1f, current(ticks): %.3f, effort: %.1f \n",elapsedTime,pivotProfile.totalTime(),startAngle,currentAngle,effortPivot);
    Serial.printf("current(ticks):%.1f,target(ticks):%.1f\n",currentAngle,targetAngle);


    if(elapsedTime > pivotProfile.totalTime()){
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
  pivot.set(0);
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
