/* 
 * File name:	log.h
 * Description:	Save and send log with usart
 */

#ifndef _LOG_H_
#define _LOG_H_

#include "stm32f103xb.h"

//Def
typedef void(*Log_Call)(void);

//Var
#define LOG_CALL_MAX 64
extern Log_Call Log_Events[LOG_CALL_MAX];
extern uint16_t Log_pSave;
extern uint16_t Log_pSend;
extern uint32_t Log_Time;
extern uint8_t Log_Enable;

//Fun
void Log_Save(Log_Call call);
void Log_SendLoop(void);
void Log_Clear(void);

#endif

//End
