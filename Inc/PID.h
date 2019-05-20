#ifndef _PID_H_
#define _PID_H_

typedef struct
{
	//Parameters
	float Kp;
	float Ki;
	float Kd;
	//value old
	float vO;
	//value integrator
	float vI;
	float limitI; //Integrator max limit
	//Delte time
	float dT;
} PID_TypeDef;
float PID_Process(PID_TypeDef *pPID, float value_e);

#endif // !_PID_H_

