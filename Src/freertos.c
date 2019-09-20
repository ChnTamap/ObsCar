/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "freertos_var.h"
#include "main.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId Task_CtrlHandle;
osThreadId Task_MPU6050Handle;

/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const *argument);
void StartTask_Ctrl(void const *argument);
void StartTaskMPU6050(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void GUI_View(void);
void GUI_Edit(uint16_t upd_items);
void GUI_Key(void);
void Key_Loop(void);
void Cali_Alpha(void);
void Step_Ready(void);
void Step_Run(void);
void Step_Obs(void);
//Log function
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of Task_Ctrl */
	osThreadDef(Task_Ctrl, StartTask_Ctrl, osPriorityNormal, 0, 128);
	Task_CtrlHandle = osThreadCreate(osThread(Task_Ctrl), NULL);

	/* definition and creation of Task_MPU6050 */
	osThreadDef(Task_MPU6050, StartTaskMPU6050, osPriorityNormal, 0, 128);
	Task_MPU6050Handle = osThreadCreate(osThread(Task_MPU6050), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const *argument)
{

	/* USER CODE BEGIN StartDefaultTask */
	uint8_t LedTime;
	TF_READY = 0x03;
	while (TF_READY)
		osDelay(200);
	printf("Im Ready!\r\nW/A:Up/Down\tA/D:Min/Add\r\nQ:PrePanel(Quit)\tE:NextPanel(Enable)\r\nL:ON/OFF Log");
	printf("R:Reset\tC:Calibrate\r\nEnter/N:ReadyRun\r\n\n\x1b[?25l");
	/* Infinite loop */
	for (;;)
	{
		osDelay(100);
		if (LedTime++ < (Flag_Mode + 1))
			GPIOC->BRR = GPIO_BRR_BR13; //Flash LED P13
		else if (LedTime > 10)
			LedTime = 0;
		osDelay(100);
		GPIOC->BSRR = GPIO_BSRR_BS13;

		//View
		if (GUI_Mode == 0)
			GUI_Key(); //This Function will be change the GUI_Mode
		if (GUI_Mode == 0)
			GUI_View();
		// else
	}
	/* USER CODE END StartDefaultTask */
}

/* StartTask_Ctrl function */
void StartTask_Ctrl(void const *argument)
{
	/* USER CODE BEGIN StartTask_Ctrl */
	//HR04
	HR04_Config();
	osDelay(1000);
	//Servo
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	/* Infinite loop */
	TF_READY &= ~0x01;
	for (;;)
	{
		osDelay(10);

		//PID
		if (Flag_Mode == FLAG_MODE_OBS)
			Step_Obs();
		else if (Flag_Mode == FLAG_MODE_RUN)
			Step_Run();
		else if (Flag_Mode == FLAG_MODE_CALI_S)
			Cali_Alpha();
		else if (Flag_Mode == FLAG_MODE_READY)
			Step_Ready();

		TIM3->CCR1 = 1500 + (value_servo - value_servo0);

		if (GUI_Mode != 0)
			GUI_Key();
		Key_Loop();
		if (KEY_IS_DOWN(0))
		{
			//Start
			Flag_Mode = FLAG_MODE_READY;
		}
		if (KEY_IS_DOWN(1))
		{
			//Inte Calibration Zero
			value_servo = 0;
			//Dead 2Second
			osDelay(2000);
			Flag_Mode = FLAG_MODE_CALI_G;
			Value_CaliGx = 0;
			Value_CaliGy = 0;
			Value_CaliGz = 0;
			Value_CaliT = 0;
		}
	}
	/* USER CODE END StartTask_Ctrl */
}

/* StartTaskMPU6050 function */
void StartTaskMPU6050(void const *argument)
{
	/* USER CODE BEGIN StartTaskMPU6050 */
	MPU_Datas datas_base;
	do
	{
		printf("\x1b[2J\x1b[0;0H*T Initing MPU6050!\r\n*");
		GPIOC->ODR ^= GPIO_ODR_ODR13; //Flash LED P13
		osDelay(200);
	} while (MPU_Init(10) != HAL_OK);
	//Calibration Zero
	printf("*T Calibration base datas...\r\n*");
	osDelay(900);
	MPU_GetIntDatas(&datas_base);
	MPU_GetFloatDatas(&data_acc, &data_gyro, &datas_base);
	MPU_GetAccelBase(&data_baseQ, &data_acc);
	printf("*T\r\nACCEL Base:%.4f %.4f %.4f      \r\n", data_acc.x, data_acc.y, data_acc.z);
	printf("GYRO  Base:%d %d %d      *\r\n", datas_base.Gx, datas_base.Gy, datas_base.Gz);
	/* Infinite loop */
	TF_READY &= ~0x02;
	for (;;)
	{
		if (TF_MPU6050)
		{
			TF_MPU6050 = 0;
			Flag_I2C = MPU_GetIntDatas(&datas);

			if (Flag_Mode == FLAG_MODE_CALI_G)
			{
				//Calibration
				Value_CaliGx += datas.Gx;
				Value_CaliGy += datas.Gy;
				Value_CaliGz += datas.Gz;
				Value_CaliT++;
				if (Value_CaliT >= 1000)
				{
					datas_base.Gx = Value_CaliGx / Value_CaliT;
					datas_base.Gy = Value_CaliGy / Value_CaliT;
					datas_base.Gz = Value_CaliGz / Value_CaliT;
					Flag_Mode = FLAG_MODE_CLOSE;
					value_servo = 0;
					data_q.x = 0;
					data_q.y = 0;
					data_q.z = 0;
					data_q.w = 1;
				}
				GPIOC->ODR ^= GPIO_ODR_ODR13; //Flash LED P13
			}
			else
			{
				datas.Gx -= datas_base.Gx;
				datas.Gy -= datas_base.Gy;
				datas.Gz -= datas_base.Gz;
				MPU_GetFloatDatas(&data_acc, &data_gyro, &datas);
				// MPU_NormVector(&data_acc);
				// MPU_QuatMulV3d(&data_acc, &data_baseQ, &data_acc);
				MPU_Filter(&data_gyro, &data_acc, &data_gEInt, &data_q, 0.0025f);
				MPU_GetAngle(&data_q, &data_angle.x, &data_angle.y, &data_angle.z);
			}

			if (TF_MPU6050 > 3)
				osDelay(3);
		}
	}
	/* USER CODE END StartTaskMPU6050 */
}

/* USER CODE BEGIN Application */
uint8_t USART1_Get(void)
{
	if (USART1->SR & USART_SR_RXNE)
	{
		USART1->SR &= ~USART_SR_RXNE;
		return USART1->DR;
	}
	return 0;
}
#define USART_IS_KEY(buf, key) ((buf == key) || (buf == (key - 'A' + 'a')))
#define USART_CLEAR printf("\r\x1b[1B\x1b[K\r\x1b[1B\x1b[K\r\x1b[1B\x1b[K\r\x1b[1B\x1b[K\x1b[1B\x1b[K\x1b[5A");
void GUI_Key(void)
{
	uint8_t buf;
	buf = USART1_Get();
	if (!buf)
		return;
	if (GUI_Mode == 0)
	{
		if (USART_IS_KEY(buf, 'E'))
		{
			//Enable Setting Mode
			GUI_Mode = 1;
			GUI_Sele = 0;
			USART_CLEAR;
			GUI_Edit(0xFFFF);
		}
		else if (USART_IS_KEY(buf, 'C'))
		{
			Flag_Mode = FLAG_MODE_CALI_G;
			Value_CaliGx = 0;
			Value_CaliGy = 0;
			Value_CaliGz = 0;
			Value_CaliT = 0;
		}
		else if (USART_IS_KEY(buf, 'R'))
		{
			Flag_Mode = 0;
			Flag_Dir = 1;
			Flag_Step = 0;
			Flag_Angle = 0;
			value_servo = 0;
			value_pDis = 0;
			data_q.x = 0;
			data_q.y = 0;
			data_q.z = 0;
			data_q.w = 1;
			printf("\x1b[?25l\x1b[2J\x1b[0;0H");
		}
		else if (USART_IS_KEY(buf, 'L'))
		{
			Log_Enable = !Log_Enable;
		}
		else if (buf == '\r' || buf == '\n' || USART_IS_KEY(buf, 'N'))
		{
			Flag_Mode = FLAG_MODE_READY;
			Flag_Step = 0;
		}
	}
	else
	{
		if (USART_IS_KEY(buf, 'Q'))
		{
			GUI_Mode = 0;
			USART_CLEAR;
		}
		else if (USART_IS_KEY(buf, 'W'))
		{
			if (GUI_Sele)
				GUI_Sele--;
			GUI_Edit(0x0003 << GUI_Sele);
		}
		else if (USART_IS_KEY(buf, 'S'))
		{
			if (GUI_Sele < Para_Len - 1)
				GUI_Sele++;
			GUI_Edit(0x0003 << (GUI_Sele - 1));
		}
		else if (USART_IS_KEY(buf, 'A'))
		{
			Para_List[GUI_Sele]--;
			GUI_Edit(0x0001 << GUI_Sele);
		}
		else if (USART_IS_KEY(buf, 'D'))
		{
			Para_List[GUI_Sele]++;
			GUI_Edit(0x0001 << GUI_Sele);
		}
	}
}
void GUI_View(void)
{
	Log_SendLoop();
	printf("\x1b[KView %c\r", Log_Enable ? 'L' : ' ');
	if (Flag_I2C == HAL_OK)
	{
		if (Flag_Mode == FLAG_MODE_CALI_G)
		{
			printf("\x1b[1BCALI:*G%d*", Value_CaliGz);
			printf(",%d", Value_CaliT);
			printf("   \r\x1b[1A");
		}
		else
		{
			printf("\x1b[1B\x1b[K*GAngleZ:%-.1f",
				   data_angle.z * 57.2957795f);
			printf(",Servo:%-d*   \r\x1b[1A",
				   value_servo * 90 / 1000);
		}
	}
	else
	{
		if (Flag_I2C == HAL_ERROR)
			printf("\x1b[1B\x1b[K*GAngleZ:ERROR");
		else if (Flag_I2C == HAL_BUSY)
			printf("\x1b[1B\x1b[K*GAngleZ:BUSY");
		else if (Flag_I2C == HAL_TIMEOUT)
			printf("\x1b[1B\x1b[K*GAngleZ:TIMEOUT");
		printf(",Servo:%-d*   \r\x1b[1A",
			   value_servo * 90 / 1000);
	}

	if (TF_HR04)
	{
		TF_HR04 = 0;
		printf("\x1b[2B\x1b[K*DHR04:%dcm  \t%dcm  *  \r\x1b[2A",
			   HR04_GetIntCm(0), HR04_GetIntCm(1));
	}
	printf("\x1b[3B\x1b[K*SNear:%02X\t%02X\t%02X\t%02X\t*	\r\x1b[3A",
		   NearSor[0], NearSor[1], NearSor[2], NearSor[3]);
	printf("\x1b[4B\x1b[K*PMode:%d,Step:%d*    \r\x1b[4A",
		   Flag_Mode, Flag_Step);
	printf("\x1b[4B\n\x1b[5A");
}
#define BG_SELECT printf("\033[46;30m")
#define BG_NORMAL printf("\033[0m")
void GUI_Edit(uint16_t upd_items)
{
	printf("Edit\r");
	if (upd_items & (0x0001 << 0))
	{
		if (GUI_Sele == 0)
			BG_SELECT;
		else
			BG_NORMAL;
		printf("\x1b[1B*G:ReadyAngle:%d  *  \r\x1b[1A",
			   Para_ANGLE_READY);
	}
	if (upd_items & (0x0001 << 1))
	{
		if (GUI_Sele == 1)
			BG_SELECT;
		else
			BG_NORMAL;
		printf("\x1b[2B*D:ObsDis:%d  *  \r\x1b[2A",
			   Para_OBS_DIS);
	}
	if (upd_items & (0x0001 << 2))
	{
		if (GUI_Sele == 2)
			BG_SELECT;
		else
			BG_NORMAL;
		printf("\x1b[3B*S:TurnAngle:%d  *  \r\x1b[3A",
			   Para_TURN_ANGLE);
	}
	if (upd_items & (0x0001 << 3))
	{
		if (GUI_Sele == 3)
			BG_SELECT;
		else
			BG_NORMAL;
		printf("\x1b[4B*P:TurnBack:%d  *  \r\x1b[4A",
			   Para_TURN_BACK);
	}
	if (upd_items & (0x0001 << 4))
	{
		if (GUI_Sele == 4)
			BG_SELECT;
		else
			BG_NORMAL;
		printf("\x1b[5B*F:DisDiff:%d  *  \r\x1b[5A",
			   Para_DIS_DIFF);
	}
	BG_NORMAL;
}
void Key_Loop(void)
{
	uint8_t i;
	for (i = 0; i < 4; i++)
	{
		NearSor[i] <<= 1;
		if (GPIOB->IDR & (GPIO_IDR_IDR3 << i))
			NearSor[i]++;
	}
	for (i = 0; i < 2; i++)
	{
		if ((KeySta[i] & 0x7F) < 0x7F)
			KeySta[i]++;
		if (GPIOA->IDR & (GPIO_IDR_IDR4 << i))
		{
			if (KEY_IS_PRESS(i))
				KeySta[i] = KEY_UP;
		}
		else
		{
			if (KEY_IS_RES(i))
				KeySta[i] = KEY_DOWN;
		}
	}
}
void Cali_Alpha(void)
{
	//Calibration value_Servo
	value_servo0 = data_angle.z * 1000 / 90;
}
#define ANGLE_READY (Para_ANGLE_READY * M_PI / 180)
void Log_StartRun(void)
{
	printf("\tStart Run\r\n");
}
//Ready
void Step_Ready(void)
{
	if (data_angle.y < ANGLE_READY && data_angle.y > -ANGLE_READY)
	{
		if (data_angle.x < ANGLE_READY && data_angle.x > -ANGLE_READY)
		{
			Flag_Mode = FLAG_MODE_RUN;
			Flag_Dir = 1;
			value_pDis = 0;
			Log_Save(Log_StartRun);
		}
	}
}
#define OBS_DIS Para_OBS_DIS
void Log_StartObs(void)
{
	printf("\tStart Obs\r\n");
}
//Run
void Step_Run(void)
{
	float tmpF, dL;
	//Is obs start?HC-SR04?NearSorL1?R1?
	if (HR04_GetIntCm(0) < OBS_DIS || NS_IS_PRESS(NearSorL1) || NS_IS_PRESS(NearSorR1))
	{
		Flag_Mode = FLAG_MODE_OBS;
		value_pDis = OBS_DIS;
		Flag_Step = 0;
		Log_Save(Log_StartObs);
		return;
	}
	//vA = A+A0
	value_angle = (data_angle.z + value_angle0);
	//Value L = D*cos(A+A0)
	value_Len = HR04_GetFloatCm(1) * cos(value_angle);
	//PID Length -> set_angle
	tmpF = set_Len - value_Len;
	dL = tmpF - pidL.vO;
	if (HR04_GetFloatCm(1) < set_Len)
		tmpF = (tmpF + set_Len - HR04_GetFloatCm(1)) / 2;
	tmpF = PID_Process(&pidL, tmpF);
	//LimitValue
	if (tmpF > 1)
		tmpF = 1;
	else if (tmpF < -1)
		tmpF = -1;
	//asin
	set_angle = asin(tmpF);
	//PID Angle -> servo alpha
	tmpF = PID_Process(&pidA, set_angle - value_angle);
	//LimitValue
	if (tmpF > 1)
		tmpF = 1;
	else if (tmpF < -1)
		tmpF = -1;
	//asin
	value_servo = asin(tmpF) * 2000.f / M_PI;
	//LimitValue
	if (value_servo > 500)
		value_servo = 500;
	else if (value_servo < -500)
		value_servo = -500;

	//Fix base
	if (dL < 1.f && dL > -1.f)
	{
		value_servo0 += value_servo / 64;
		value_angle0 -= (data_angle.z + value_angle0) / 64;
	}
}
uint8_t Log_TurnBack_value = 0;
void Log_TurnBack(void)
{
	if (Log_TurnBack_value)
		printf("\tTurn back:DisDiff,FlagAngle");
	else
		printf("\tTurn back:NearSor,FlagAngle");
	printf(":%d\r\n", Flag_Angle * 90 / 1000);
}
uint8_t Log_TurnAngle_value = 0;
void Log_TurnAngle(void)
{
	if (Log_TurnAngle_value)
		printf("\tTurn Left\r\n");
	else
		printf("\tTurn Right\r\n");
}
//Obs
void Step_Obs(void)
{
	if (Flag_Step == 0)
	{
		if (Flag_Dir)
		{
			//Left
			Flag_Angle = -TURN_ANGLE;
			NearSorX1 = &(NearSorL1);
			NearSorX2 = &(NearSorL2);
			Log_TurnAngle_value = 0; //Log
		}
		else
		{
			//Right
			Flag_Angle = TURN_ANGLE;
			NearSorX1 = &(NearSorR1);
			NearSorX2 = &(NearSorR2);
			Log_TurnAngle_value = 1; //Log
		}
		Log_Save(Log_TurnAngle); //Log
		Flag_Step++;
	}
	if (Flag_Step == 1)
	{
		//判断突变
		if ((HR04_GetFloatCm(0) - value_pDis > DIS_DIFF))
		{
			Log_TurnBack_value = 1;
			Log_Save(Log_TurnBack);
			Flag_Angle = Flag_Dir ? TURN_BACK : -TURN_BACK;
		}
		if ((NS_IS_UP(*NearSorX1)))
		{
			Log_TurnBack_value = 0;
			Log_Save(Log_TurnBack);
			Flag_Angle = Flag_Dir ? TURN_BACK : -TURN_BACK;
		}
		//判断准备经过障碍
		if (NS_IS_PRESS(*NearSorX2))
		{
			Flag_Step++;
		}
		value_pDis = HR04_GetFloatCm(0);
		//避障
		value_servo = Flag_Angle;
	}
	else if (Flag_Step == 2)
	{
		//判断超过障碍
		if (NS_IS_RES(*NearSorX2))
		{
			//切换方向
			Flag_Step = 0;
			Flag_Dir = !Flag_Dir;
			value_pDis = 400;
		}
	}
	//判断LR1障碍
	if (NS_IS_PRESS(NearSorL1) && NS_IS_PRESS(NearSorR1))
		value_servo = Flag_Angle;
	else if (NS_IS_PRESS(NearSorL1))
		value_servo = -TURN_ANGLE;
	else if (NS_IS_PRESS(NearSorR1))
		value_servo = TURN_ANGLE;
	else if (Flag_Step == 2)
		value_servo = 0;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
