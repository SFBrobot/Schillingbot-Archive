#pragma platform(VEX)
#pragma competitionControl(Competition);

void init();
void endAuton();
void endUserOp();
task auton();
task userOp();

task main() {
	init();

	while(true) {
		while(bIfiRobotDisabled) { wait1Msec(25); EndTimeSlice(); }

		if(bIfiAutonomousMode) {
			startTask(auton);

			while(bIfiAutonomousMode && !bIfiRobotDisabled) { wait1Msec(25); EndTimeSlice(); }

			stopTask(auton);
			endAuton();
		}
		else {
		  startTask(userOp);

		  while(!bIfiAutonomousMode && !bIfiRobotDisabled) { wait1Msec(25); EndTimeSlice(); }

		  stopTask(userOp);
		  endUserOp();
		}
	}
}
