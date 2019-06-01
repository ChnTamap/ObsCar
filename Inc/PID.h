#ifndef _PID_H_
#define _PID_H_

typedef struct
{
	//Parameters
	float Kp;
	float Ki;
	float Kd;
	//Delte time
	float dT;
	//value integrator
	float vI;
	float limitI; //Integrator max limit
	//value old
	float vO;
} PID_TypeDef;
float PID_Process(PID_TypeDef *pPID, float value_e);

#endif // !_PID_H_

