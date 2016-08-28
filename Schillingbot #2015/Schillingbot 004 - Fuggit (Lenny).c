#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  armLim,         sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           flWheel,       tmotorVex393TurboSpeed_HBridge, openLoop, driveRight)
#pragma config(Motor,  port2,           frWheel,       tmotorVex393TurboSpeed_MC29, openLoop, reversed, driveLeft)
#pragma config(Motor,  port3,           blArm,         tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port4,           tlArm,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           rIntake,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lIntake,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           brArm,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           trArm,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           brWheel,       tmotorVex393_MC29, openLoop, reversed, driveLeft, encoderPort, I2C_3)
#pragma config(Motor,  port10,          blWheel,       tmotorVex393_HBridge, openLoop, driveRight, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "rkUtil001.h"
#include "rkCompetition001.h"

#define masterLWheel blWheel
#define masterRWheel brWheel
#define masterArm blArm

#define setLDrive(val) (motor[flWheel] = motor[blWheel] = val)
#define setRDrive(val) (motor[frWheel] = motor[brWheel] = val)
#define setDrive(val) setLDrive(setRDrive(val))
#define setPointTurn(val) setRDrive(-setLDrive(val));

#define setArms(val) motor[blArm] = motor[tlArm] = motor[brArm] = motor[trArm] = val
#define setIntake(val) motor[lIntake] = motor[rIntake] = val

//Cheater-cube autonomous subroutine
void autonChCube() {
  setArms(63);
  wait1Msec(1000);

  setArms(-127);

  block(!SensorValue[armLim]);
}

task autonLineAlignIntake() {
  setIntake(63);

  wait1Msec(30);

  setIntake(0);
}

//Blue-post straight-line autonomous subroutine
void autonLineRed() {
  startTask(autonLineAlignIntake);

  setPointTurn(63);

  block(nMotorEncoder[masterLWheel] <= 88);

  setDrive(0);

  setDrive(127);

  block(nMotorEncoder[masterLWheel] <= 300);

  setDrive(-5);

  wait1Msec(100);

  setDrive(0);

  wait1Msec(100);

  setIntake(127);

  wait1Msec(1000);

  setIntake(0);

  setRDrive(127);

  block(nMotorEncoder[masterRWheel] <= 700);

  setRDrive(0);

  nMotorEncoder[masterLWheel] = 0;

  setLDrive(-95);

  block(nMotorEncoder[masterLWheel] >= -100);

  nMotorEncoder[masterLWheel] =
    nMotorEncoder[masterRWheel] = 0;

  setDrive(95);

  block(nMotorEncoder[masterLWheel] <= 200);

  setDrive(-10);

  wait1Msec(100);

  setDrive(0);

  wait1Msec(100);

  setIntake(127);

  wait1Msec(750);

  setIntake(0);
}

void init() {
  clearLCDLine(0);

  nMotorEncoder[masterLWheel] =
    nMotorEncoder[masterRWheel] = 0;

  setArms(-31);

  block(!SensorValue[armLim]);

  setArms(0);
}

task auton() {
  autonChCube();

  autonLineRed();

  endAuton();
}

void endAuton() {
  motor[flWheel] = motor[frWheel] = setArms(0);
}

void endUserOp() {
}

task userOp() {
  short digiVelArm, digiVelIntake;
  while (true) {
  motor[frWheel] = motor[brWheel] = abs(vexRT[Ch2]) > 3 ? vexRT[Ch2] : 0;
  motor[flWheel] = motor[blWheel] = abs(vexRT[Ch3]) > 3 ? vexRT[Ch3] : 0;

  digiVelArm = vexRT[Btn7D] ? 63 : 127;
  digiVelIntake = vexRT[Btn8D] ? 63 : 127;

  setArms((vexRT[Btn5U] ^ vexRT[Btn5D]) ? (vexRT[Btn5U] ? digiVelArm : SensorValue[armLim] ? 0 : -digiVelArm) : SensorValue[armLim] ? -10 : 20);

    motor[lIntake] = motor[rIntake] =
  (vexRT[Btn6U] ^ vexRT[Btn6D]) ? (vexRT[Btn6U] ? digiVelIntake : -digiVelIntake) : 0;
  }
}
