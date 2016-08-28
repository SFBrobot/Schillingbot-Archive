#pragma config(Motor,  port1,           L1,            tmotorVex393_HBridge, openLoop, reversed, driveLeft)
#pragma config(Motor,  port2,           L2,            tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port3,           L3,            tmotorVex393_MC29, openLoop, reversed, driveLeft)
#pragma config(Motor,  port4,           L4,            tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port5,           R1,            tmotorVex393_MC29, openLoop, driveRight)
#pragma config(Motor,  port6,           R2,            tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port7,           R3,            tmotorVex393_MC29, openLoop, driveRight)
#pragma config(Motor,  port8,           R4,            tmotorVex393_MC29, openLoop, reversed, driveRight)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
  // Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{
  // .....................................................................................
  // Insert user code here.
  // .....................................................................................

}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

long time, lTime = 0;
float dt, left, right, lLeft = 0, lRight = 0;

const float thRate = 63./1000;

float max(float a, float b) { return a > b ? a : b; }
float min(float a, float b) { return a < b ? a : b; }

task usercontrol()
{
	clearTimer(T1);
	// User control code here, inside the loop

	while (true)
	{
		time = time1[T1];
		dt = time - lTime;

		left = min(abs(vexRT[Ch3]), abs(lLeft) + thRate) * sgn(vexRT[Ch3]);
		right = min(abs(vexRT[Ch2]), abs(lRight) + thRate) * sgn(vexRT[Ch2]);

		motor[L1] =
			motor[L2] =
			motor[L3] =
			motor[L4] = (short)max(-127, min(127, left));

		motor[R1] =
			motor[R2] =
			motor[R3] =
			motor[R4] = (short)max(-127, min(127, right));

		lTime = time;
		lLeft = left;
		lRight = right;
	}
}