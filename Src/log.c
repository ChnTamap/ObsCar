#include "log.h"

Log_Call Log_Events[LOG_CALL_MAX];
uint8_t Log_pSave = 0;
uint8_t Log_pSend = 0;
uint32_t Log_Time = 0;
uint8_t Log_Enable = 0;

//Fun
void Log_Save(Log_Call call)
{
	if (!Log_Enable)
	{
		Log_pSave = 0;
		return;
	}
	if (Log_pSave < LOG_CALL_MAX)
	{
		Log_Events[Log_pSave] = call;
		Log_pSave++;
	}
}

void Log_SendLoop(void)
{
	if (Log_Enable)
	{
		for (Log_pSend = 0; Log_pSend < Log_pSave; Log_pSend++)
		{
			printf("\r\x1b[K%d", Log_Time);
			Log_Events[Log_pSend]();
		}
	}
	Log_pSave = 0;
}

//End
