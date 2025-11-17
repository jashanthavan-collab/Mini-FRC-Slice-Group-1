#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// Motors.

NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);
NoU_Motor intakeMotor(5);
NoU_Motor elevator(6);

// Servos.
NoU_Servo stageI(1);
NoU_Servo stageII(2);
NoU_Servo clawServo(3);

// Drivetrain.
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
  PestoLink.begin("Robot Name");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_ENABLED);

  frontLeftMotor.setBrakeMode(true);
  frontRightMotor.setBrakeMode(true);
  backLeftMotor.setBrakeMode(true);
  backRightMotor.setBrakeMode(true);
}

void loop() {
  chassis(); 
  elevator();
  intake();

  intakeMotor.set(intakeT);
  stageI.write(angleI);
  stageII.write(angleII);
  clawServo.write(clawAngle);
}

void chassis() {
    static unsigned long lastPrintTime = 0;
    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.roll * angular_scale );
        lastPrintTime = millis();
    }

    // This measures your batteries voltage and sends it to PestoLink
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = PestoLink.getAxis(1);
        float fieldPowerY = -1 * PestoLink.getAxis(0);
        float rotationPower = -1 * PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.roll * angular_scale;

        // Rotate joystick vector to be robot-centric
        float cosA = cos(heading);
        float sinA = sin(heading);

        float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
        float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        drivetrain.holonomicDrive(0, 0, 0); // stop motors if connection is lost

        NoU3.setServiceLight(LIGHT_DISABLED);
    }
}

void elevator() {
  if(PestoLink.buttonHeld(0)) {
    elevator.set(1); 
  } else if(PestoLink.buttonHeld(1)) {
    elevator.set(-1); 
  } else {
    elevator.set(0); }}


  
