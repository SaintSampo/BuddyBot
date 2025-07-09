#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "AutoModeAgent.h"
#include "OpticalFlowAgent.h"
#include "DrivetrainHack.h"
#include "Automodes.h"

NoU_Motor frontLeftMotor(7);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);

NoU_Motor elevator(5);
NoU_Motor pivot(4);
NoU_Motor intake(3);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

typedef enum {
  STATE_ALGEA = 0,
  STATE_CORAL = 1,
  STATE_LOW = 2,
  STATE_HIGH = 3,
  TOGGLE_ENABLE = 4,
  STATE_SCORE = 5,
  STATE_INTAKE = 6,
  STATE_APPROACH = 7,
  STATE_DEALGEA = 9
} pestoButtons;

ActuatorControl controlState = {
  .coralMode = false,
  .highMode = false,
  
  .enableDrivetrain = false,
  .enableActuation = false,

  .targetPose = {0,0,0},
  .targetVelocity {0,0,0},
  .elevatorTarget = 0.0f,
  .pivotTarget = 0.0f,
  .intakePower = 0.0f
};

QueueHandle_t AutoQueue;

void setup() {
  Serial.begin(115200);
  PestoLink.begin("BuddyBot");

  pinMode(0, INPUT_PULLUP);

  NoU3.begin();

  pivot.beginEncoder();
  elevator.beginEncoder();
  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(true);
  elevator.setInverted(true);
  pivot.setInverted(true);
  intake.setInverted(true);
  //elevator.setBrakeMode(true);
  drivetrain.setMotorCurves(0.3, 1, 0.05, 2.5);
  
  //controlState.enableActuation = true;
  AutoModeAgent_beginControlTask(&controlState);
  AutoQueue = xQueueCreate(10, sizeof(MotionProfile));

  OpticalFlow_begin();
  NoU3.calibrateIMUs(); // takes 1000ms
}

float measured_angle = 55.0;
float angular_scale = (10.0 * TWO_PI) / measured_angle;

void loop() {
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()) {
    //Serial.printf("yaw(rad):%.3f,pitch(rad):%.3f,roll(rad):%.3f\r\n", NoU3.yaw, NoU3.pitch, NoU3.roll);
    //PestoLink.printfTerminal("elevator:%.3d, pivot:%.3d\r\n",elevator.getPosition(),pivot.getPosition());
    PestoLink.printfTerminal("X(m):%.3f, Y(m):%.3f, theta(rad):%.3f\r\n", OpticalFlow_getX(), OpticalFlow_getY(),  OpticalFlow_getTheta());
    lastPrintTime = millis();
  }

  // Measures battery voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  // Here we decide what the throttle and rotation direction will be based on gamepad inputs
  if (PestoLink.isConnected()) {
    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -1 * PestoLink.getAxis(1);
    float rotationPower = -1 * PestoLink.getAxis(2);
    
    // Get robot heading (in radians) from a gyro
    //float heading = OpticalFlow_getTheta();
    float heading = NoU3.yaw * angular_scale;

    // Rotate joystick vector to be field-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;
    
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
    
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    NoU3.setServiceLight(LIGHT_DISABLED);
  }

  if (PestoLink.keyHeld(Key::Q) || !digitalRead(0)) {
    delay(2000);

    xQueueSend(AutoQueue, &midwayWP, 0);
    xQueueSend(AutoQueue, &lineWP, 0);
    //PestoLink.printfTerminal("starting automode setup");

    AutoModeAgent_begin(AutoQueue);

    controlState.enableActuation = false;
  }

  if (PestoLink.keyHeld(Key::W) ) {
    xQueueSend(AutoQueue, &midwayWP, 0);
    xQueueSend(AutoQueue, &reef1WP, 0);
    xQueueSend(AutoQueue, &approach1HighWP, 0);
    xQueueSend(AutoQueue, &score1HighWP, 0);
    xQueueSend(AutoQueue, &loadingWP, 0);
    xQueueSend(AutoQueue, &reef2WP, 0);
    xQueueSend(AutoQueue, &approach2HighWP, 0);
    xQueueSend(AutoQueue, &score2HighWP, 0);
    xQueueSend(AutoQueue, &endWP, 0);
    //PestoLink.printfTerminal("starting automode");

    AutoModeAgent_begin(AutoQueue);
  }

  if(PestoLink.buttonHeld(STATE_LOW)){
    controlState.highMode = false;
    PestoLink.printfTerminal("state low");
  }
  if(PestoLink.buttonHeld(STATE_HIGH)){
    controlState.highMode = true;
    PestoLink.printfTerminal("state high");
  }
  if(PestoLink.buttonHeld(STATE_ALGEA)){
    controlState.coralMode = false;
    PestoLink.printfTerminal("state algea");
  }
  if(PestoLink.buttonHeld(STATE_CORAL)){
    controlState.coralMode = true;
    PestoLink.printfTerminal("state coral");
  }

  if(PestoLink.buttonHeld(STATE_APPROACH)){
    if(controlState.coralMode){
      if(controlState.highMode) {
        controlState.elevatorTarget = 1400;
        controlState.pivotTarget = 590;
        controlState.intakePower = -0.8;
      } else {
        controlState.elevatorTarget = 265;
        controlState.pivotTarget = 626;
        controlState.intakePower = -0.8;
      }
    } else { //algea mode
      if(controlState.highMode) {
        controlState.elevatorTarget = 1740.0*(16.0/18.0);
        controlState.pivotTarget = 726;
        controlState.intakePower = -0.8;
      } else {
        controlState.elevatorTarget = 0;
        controlState.pivotTarget = 290;
        controlState.intakePower = -0.8;
      }
    }
  }

  static bool lastStateIntake = false;
  static bool lastStateScore = false;
  static bool lastToggleEnable = false;
  bool stateIntake = PestoLink.buttonHeld(STATE_INTAKE);
  bool stateScore = PestoLink.buttonHeld(STATE_SCORE);
  bool toggleEnable = PestoLink.buttonHeld(TOGGLE_ENABLE);

  //toggle enable pressed
  if(toggleEnable && !lastToggleEnable) {
    if(controlState.enableActuation){
      PestoLink.printfTerminal("disabling actuators");
      controlState.enableActuation = false;
    } else {
      PestoLink.printfTerminal("enabling actuators");
      controlState.enableActuation = true;
    }
  }

  if(stateScore){
    if(controlState.coralMode){
      if(controlState.highMode) {
        controlState.elevatorTarget = 1450;
        controlState.pivotTarget = 680;
        controlState.intakePower = 1;
      } else {
        controlState.elevatorTarget = 250.0*(16.0/18.0);
        controlState.pivotTarget = 674;
        controlState.intakePower = 1;
      }
    } else { //algea mode
      if(controlState.highMode) {
        controlState.elevatorTarget = 1740.0*(16.0/18.0);
        controlState.pivotTarget = 553;
        controlState.intakePower = 1;
      } else {
        controlState.elevatorTarget = 0;
        controlState.pivotTarget = 290;
        controlState.intakePower = 1;
      }
    }
  }

  //algea intake approach
  if(PestoLink.buttonHeld(STATE_DEALGEA)){
    controlState.elevatorTarget = 546;
    controlState.pivotTarget = 877;
    controlState.intakePower = -0.8;
    PestoLink.printfTerminal("dealgify");
  }

  //intake pressed
  if(stateIntake && !lastStateIntake) {
    if(controlState.coralMode){
      PestoLink.printfTerminal("Intaking Coral");
      controlState.elevatorTarget = 585;
      controlState.pivotTarget = 145;
      controlState.intakePower = -0.8;
    } else { //algea mode
      PestoLink.printfTerminal("Intaking High Algae");
      controlState.elevatorTarget = 1300;
      controlState.pivotTarget = 800;
      controlState.intakePower = -0.8;
    }
  }

  //stow position, intake released, or score released
  if((!stateIntake && lastStateIntake) || (!stateScore && lastStateScore)) {
    if(controlState.coralMode){
      controlState.elevatorTarget = 0;
      controlState.pivotTarget = 50;
      controlState.intakePower = -0.8;
    } else { //algea mode
      controlState.elevatorTarget = 0;
      controlState.pivotTarget = 230;
      controlState.intakePower = -0.8;
    }
  }

  lastStateIntake = stateIntake;
  lastStateScore = stateScore;
  lastToggleEnable = toggleEnable;


}

//#include <stdio.h>
//#include "driver/ledc.h"
//#include "esp_err.h"

// #define LEDC_TIMER              LEDC_TIMER_3
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (5) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_1_BIT // Set duty resolution
// #define LEDC_DUTY               (1) // Set duty as a percentage of res
// #define LEDC_FREQUENCY          (30000000) // Frequency in Hertz. Max is 40 MHz

// void configTimer(){

//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .timer_num        = LEDC_TIMER,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
//         .clk_cfg          = LEDC_AUTO_CLK,
//         .deconfigure      = false
//     };
//     ledc_timer_config(&ledc_timer);

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .timer_sel      = LEDC_TIMER,
//         .duty           = 0, // Set duty to 0%
//         .hpoint         = 0,
//         .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
//         .flags = {
//           .output_invert = 0                         // No output inversion
//         }
//     };
//     ledc_channel_config(&ledc_channel);

//     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
//     ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
// }