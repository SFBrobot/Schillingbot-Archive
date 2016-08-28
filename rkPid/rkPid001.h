#include "rkUtil001.h"

typedef struct {
  tMotor motor; //Master motor port
  bool useMotorEncoder, isOnTarget; //Use I2C? and is motor within threshold
  tSensors enc; //Encoder connected to motor
  short setPoint, thresh, kI, lastError, moveLimit, limit; //Rotation setpoint, threshold, I coefficient, last error value (for derivative), speed limit, and moving speed limit (for locking)
  long lastTime; //Last time (for derivative)
  float kP, kD; //P and D coefficients
  bool doRun,; //Should we update this?
} Pid;

short getPidEncoder(Pid* pid) { return pid->useMotorEncoder ? nMotorEncoder[pid->motor] : SensorValue[pid->enc]; }

void clearPidEncoder(Pid* pid) {
  if(pid->useMotorEncoder) nMotorEncoder[pid->motor] = 0;
  else SensorValue[pid->enc];
}

short getPidError(Pid* pid) {
  short error = pid->setPoint - getPidEncoder(pid);
  pid->isOnTarget = abs(error) <= pid->thresh;
  return pid->isOnTarget ? 0 : error;
}

short calcPid(Pid* pid) {
  short error = getPidError(pid);

  if(error == 0) return 0;

  short mtrPwr = 0;
  long time = time1[T4];

  mtrPwr = max(-pid->limit, min(pid->limit, mtrPwr + pid->kP * error)); //Calculate proportional power
  mtrPwr = max(-pid->limit, min(pid->limit, mtrPwr + sgn(error) * pid->kI)); //Calculate integral power
  if(time > pid->lastTime) mtrPwr = max(-pid->limit, min(pid->limit, mtrPwr + pid->kD * (float)(pid->lastError - error) / (float)(time - pid->lastTime))); //Calculate derivative power

  pid->lastError = error;
  pid->lastTime = time;

  return mtrPwr;
}

short updatePid(Pid* pid) { return motor[pid->motor] = calcPid(pid); }

void initPid(Pid* pid, tMotor motor, bool useMotorEncoder, short thresh, float kP, short kI, float kD) {
  pid->motor = motor;
  pid->useMotorEncoder = useMotorEncoder;
  if(!useMotorEncoder) pid->enc = getEncoderForMotor(motor);
  pid->setPoint = 0;
  pid->lastError = 0;
  pid->limit = pid->moveLimit = 127;
  pid->lastTime = 0;
  pid->doRun = false;
  pid->thresh = thresh;
  pid->kP = kP;
  pid->kI = kI;
  pid->kD = kD;
}

Pid driveLPid, driveRPid, armPid;

bool isPidRunning = false;
task updatePids();

void beginPid(Pid* pid, bool doReset = false) {
  if(doReset) {
    clearPidEncoder(pid);
    pid->setPoint = 0;
  }
  if(!isPidRunning) {
    isPidRunning = true;
    startTask(updatePids);
    clearTimer(T4);
  }
  pid->doRun = true;
	pid->isOnTarget = false;
  pid->lastTime = time1[T4];
  pid->lastError = getPidError(pid);
}

void lockPid(Pid* pid, short limit = 127) {
  pid->limit = limit;
  pid->setPoint = getPidEncoder(pid);
  beginPid(pid);
}

void movePid(Pid* pid, short setPoint) {
  pid->limit = pid->moveLimit;
	pid->setPoint = setPoint;
	beginPid(pid);
}

void movePid(Pid* pid, short setPoint, short limit) {
	pid->limit = pid->moveLimit = limit;
	pid->setPoint = setPoint;
	beginPid(pid, setPoint);
}

void movePidBy(Pid* pid, short by) { movePid(pid, pid->setPoint + by); }

void movePidBy(Pid* pid, short by, short limit) { movePid(pid, pid->setPoint + by, limit); }

void stopPid(Pid* pid) {
	if(pid->doRun) {
  	pid->doRun = false;
  }
  motor[pid->motor] = 0;
}

void stopAllPids() { isPidRunning = false; }
