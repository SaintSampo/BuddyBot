#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include "Wire.h"
#include "AutoModeAgent.h"
#include "OpticalFlowAgent.h"
#include "DrivetrainHack.h"

NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(6);
NoU_Motor rearLeftMotor(4);
NoU_Motor rearRightMotor(5);

NoU_Servo servo(1);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

MotionProfile motionProfile = {
    .x = { .target = 0.300, .startRate = 0, .endRate = 0, .maxAbsoluteRate = 0.050, .maxAccel = 20 },
    .y = { .target =   0, .startRate = 0, .endRate = 0, .maxAbsoluteRate =   0.1, .maxAccel =   3.2 },
    .theta = { .target =   0, .startRate = 0, .endRate = 0, .maxAbsoluteRate =   0.3, .maxAccel =   3.2 }
};

void setup() {
    Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);
    NoU3.beginMotors();
    NoU3.beginServiceLight();

    PestoLink.begin("BuddyBot");
    Serial.begin(115200);

    OpticalFlow_begin();

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    frontLeftMotor.setMotorCurve(0.25, 1, 0.2, 1);
    frontRightMotor.setMotorCurve(0.25, 1, 0.2, 1);
    rearLeftMotor.setMotorCurve(0.25, 1, 0.2, 1);
    rearRightMotor.setMotorCurve(0.25, 1, 0.2, 1);
    
    AutoModeAgent_holdStability(true, true, true);
}

void loop() {

    // This measures your batteries voltage and sends it to PestoLink
    // You could use this value for a lot of cool things, for example make LEDs flash when your batteries are low?
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    // Here we decide what the throttle and rotation direction will be based on gamepad inputs   
    if (PestoLink.update()) {
        float yVelocity = -PestoLink.getAxis(1);
        float xVelocity = PestoLink.getAxis(0);
        float rotation = PestoLink.getAxis(2);

        // Get robot heading (in radians) from a gyro
        float heading = -OpticalFlow_getTheta();

        // Rotate joystick vector to be field-centric
        float cosA = cos(heading);
        float sinA = sin(heading);

        float xField = xVelocity * cosA - yVelocity * sinA;
        float yField = xVelocity * sinA + yVelocity * cosA;

        drivetrain.holonomicDrive(xField, yField, rotation);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    if(PestoLink.buttonHeld(0)){
        Serial.println("starting Automode");
        AutoModeAgent_executeProfile(motionProfile);
    }

    // No need to mess with this code
    NoU3.updateServiceLight();
}
