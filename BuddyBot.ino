#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "AutoModeAgent.h"
#include "OpticalFlowAgent.h"
#include "DrivetrainHack.h"

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

NoU_Motor frontLeftMotor(7);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);

NoU_Motor pivot(4);
NoU_Motor elevator(5);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);


#define LEDC_TIMER              LEDC_TIMER_3
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_1_BIT // Set duty resolution
#define LEDC_DUTY               (1) // Set duty as a percentage of res
#define LEDC_FREQUENCY          (30000000) // Frequency in Hertz. Max is 40 MHz

void configTimer(){

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {
          .output_invert = 0                         // No output inversion
        }
    };
    ledc_channel_config(&ledc_channel);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}



void setup() {

  pinMode(0, INPUT_PULLUP);
  //configTimer();

  Wire.begin(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC, 400000);
  Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);

  PestoLink.begin("BuddyBot");
  Serial.begin(115200);

  pca9685.setupSingleDevice(Wire1, 0x40);
  pca9685.setupOutputEnablePin(12);
  pca9685.enableOutputs(12);
  pca9685.setToFrequency(1526);

  NoU3.beginIMUs();
  NoU3.beginServiceLight();

  pivot.beginEncoder();
  elevator.beginEncoder();

  NoU3.calibrateIMUs();
  OpticalFlow_begin();

  frontLeftMotor.setInverted(true);
  rearLeftMotor.setInverted(true);

  elevator.setInverted(true);
  elevator.setBrakeMode(true);

  drivetrain.setMotorCurves(0.25, 1, 0.2, 1);

  //AutoModeAgent_holdStability(false, false, false);
  //pivot_holdStability();
  //Pivot_executeProfile(pivotProfile);
}

float measured_angle = 55.0;
float angular_scale = (10.0 * TWO_PI) / measured_angle;

unsigned long lastPrintTime = 0;
void loop() {

  if (lastPrintTime + 100 < millis()) {
    //Serial.printf("yaw(rad):%.3f,pitch(rad):%.3f,roll(rad):%.3f\r\n", NoU3.yaw, NoU3.pitch, NoU3.roll);
    //Serial.printf("optical(rad):%.3f,gyro(rad):%.3f\r\n", OpticalFlow_getTheta(), NoU3.yaw * 1.145);
    //Serial.printf("optical(rad):%.3f,difference(rad):%.3f\r\n", OpticalFlow_getTheta(), OpticalFlow_getTheta() /(NoU3.yaw * (PI/180.0)));
    //PestoLink.printfTerminal("yaw(rad):%.3f,adjusted_yaw(rad):%.3f\r\n",NoU3.yaw,NoU3.yaw*angular_scale);
    //PestoLink.printfTerminal("yaw(rad):%.3f,adjusted_yaw(rad):%.3f\r\n",NoU3.yaw,NoU3.yaw*angular_scale);
    lastPrintTime = millis();
  }

  // This measures your batteries voltage and sends it to PestoLink
  // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  //PestoLink.printTerminal("optical(rad):,");
  //PestoLink.printfTerminal("optical(rad):%.3f,gyro(rad):%.3f\r\n", OpticalFlow_getTheta(), NoU3.yaw * 1.145);
  // Here we decide what the throttle and rotation direction will be based on gamepad inputs
  if (PestoLink.isConnected()) {
    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -PestoLink.getAxis(1);
    float rotationPower = -PestoLink.getAxis(2);

    if (PestoLink.buttonHeld(1)) rotationPower += -1;
    if (PestoLink.buttonHeld(2)) rotationPower += 1;
    

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


  if (PestoLink.buttonHeld(0) || !digitalRead(0)) {
    Serial.println("starting Automode");
    //AutoModeAgent_executeProfile(motionProfile);
    //AutoModeAgent_begin();
    while(true){
      Pose fieldTargetPose = {0,0,0};
      Pose fieldTargetVelocity = {0,0,0};
      drivetrain_set(fieldTargetPose,fieldTargetVelocity);
    }
  }

  static float elevatorTarget = 0;
  static float pivotTarget = 0;

  if(PestoLink.buttonHeld(3))       elevatorTarget = 1800;
  else if (PestoLink.buttonHeld(1)) elevatorTarget = 900;
  else if (PestoLink.buttonHeld(0)) elevatorTarget = 0;

  if(PestoLink.buttonHeld(6))       pivotTarget = 800;
  else if (PestoLink.buttonHeld(5)) pivotTarget = 400;
  else if (PestoLink.buttonHeld(4)) pivotTarget = 0;

  elevator_set(elevatorTarget);
  pivot_set(pivotTarget);
}
