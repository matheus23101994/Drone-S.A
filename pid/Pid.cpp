#include "Pid.h"

// Pid::Pid(float _Kp, float _Ki, float _Kd)
// {	
//     Kp = _Kp;
// 	Ki = _Ki; 
// 	Kd = _Kd;
// }

void Pid::setGains(float _Kp, float _Ki, float _Kd)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

void Pid::setSetPoint(float newSetPoint)
{
	setPoint =  newSetPoint;
}

void Pid::setOutputLimits(float min, float max)
{
	if(min > max) return;
	outMin = min;
	outMax = max;
}

float Pid::process(float newValue)
{
	float error = setPoint - newValue;
 	float res;

	Iterm += (Ki * error);
	if     (Iterm > outMax) Iterm = outMax;
	else if(Iterm < outMin) Iterm = outMin;
	float Dinput = (newValue - lastValue);
	// Compute PID OutPut
	res = Kp * error + Iterm + Kd * Dinput;
	if     (res > outMax) res = outMax;
	else if(res < outMin) res = outMin;
	lastValue = newValue;

	return (res);
}

