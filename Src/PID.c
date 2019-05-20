#include "PID.h"

float PID_Process(PID_TypeDef *pPID, float value_e)
{
	float value_out;

	//Integrator
	pPID->vI += value_e * pPID->dT;
	if (pPID->vI > pPID->limitI)
		pPID->vI = pPID->limitI;
	else if (pPID->vI < -pPID->limitI)
		pPID->vI = -pPID->limitI;

	//PID
	value_out = value_e * pPID->Kp;
	value_out += pPID->vI * pPID->Ki;
	value_out += (value_e - pPID->vO) * pPID->Kd / pPID->dT;

	return value_out;
}


