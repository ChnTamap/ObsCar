#ifndef _HR04_H_
#define _HR04_H_

/* Inc */
#include "tim.h"

/* Var */
extern uint8_t TF_HR04;
extern uint16_t HR04_Value[2];

/* Fun */
void HR04_Config(void);
void HR04_IRQHandler(void);
#define HR04_GetIntCm(index) ((HR04_Value[index]) * 17)
#define HR04_GetFloatCm(index) (((float)HR04_Value[index]) * 17)

#endif // !_HR04_H_
