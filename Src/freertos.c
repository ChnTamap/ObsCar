/**
	******************************************************************************
	* File Name					: freertos.c
	* Description				: Code for freertos applications
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
	*		this list of conditions and the following disclaimer.
	* 2. Redistributions in binary form must reproduce the above copyright notice,
	*		this list of conditions and the following disclaimer in the documentation
	*		and/or other materials provided with the distribution.
	* 3. Neither the name of STMicroelectronics nor the names of other 
	*		contributors to this software may be used to endorse or promote products 
	*		derived from this software without specific written permission.
	* 4. This software, including modifications and/or derivative works of this 
	*		software, must execute solely and exclusively on microcontroller or
	*		microprocessor devices manufactured by or for STMicroelectronics.
	* 5. Redistribution and use of this software other than as permitted under 
	*		this license is void and will automatically terminate your rights under 
	*		this license. 
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
#include "tim.h"
#include "MPU6050.h"
#include "HR04.h"
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId Task_CtrlHandle;
osThreadId Task_MPU6050Handle;

/* USER CODE BEGIN Variables */
//#Math
#define M_PI 3.14159265358f
//#Data
uint8_t TF_READY = 0xFF;
uint8_t TF_MPU6050 = 0;
enum
{
	FLAG_MODE_CLOSE = 0,
	FLAG_MODE_CALI_G, //¾²Ì¬Ð£ÁãÍÓÂÝÒÇGyro
	FLAG_MODE_READY,  //×¼±¸»¬ÐÐ
	FLAG_MODE_CALI_S, //¶¯Ì¬Ð£Áã¶æ»úvalue_Servo
	FLAG_MODE_CALI_T, //¶¯Ì¬ÐÞÕý¾ø¶Ô½Ç¶ÈTheta
	FLAG_MODE_RUN,	//ÎÈ¶¨»¬ÐÐ½×¶Î
	FLAG_MODE_OBS	 //±ÜÕÏ½×¶Î
};
uint8_t Flag_Mode = 0;
int32_t Value_CaliGx, Value_CaliGy, Value_CaliGz;
uint16_t Value_CaliT = 0;
uint16_t EncodeSor[2] = {0, 0};
uint8_t NearSor[4];
MPU_Datas datas;
MPU_V3D data_acc;
MPU_V3D data_gyro;
MPU_V3D data_angle;
MPU_V3D data_gEInt;
MPU_Quat data_q = {0, 0, 0, 1};
MPU_Quat data_baseQ = {0, 0, 0, 1};
#define KEY_DOWN 0x80
#define KEY_UP 0x00
#define KEY_LONG (KEY_DOWN + 60)
#define KEY_IS_DOWN(i) (KeySta[i] == KEY_DOWN)
#define KEY_IS_PRESS(i) (KeySta[i] >= KEY_DOWN)
#define KEY_IS_UP(i) (KeySta[i] == KEY_UP)
#define KEY_IS_RES(i) (KeySta[i] < KEY_DOWN)
#define KEY_IS_LDOWN(i) (KeySta[i] == KEY_LONG)
#define KEY_IS_LPRESS(i) (KeySta[i] >= KEY_LONG)
uint8_t KeySta[2];
//#PID
/*
HR04_GetFloatCm -> vL
data_angle.z -> vTheta
*/
PID_TypeDef pidL = {1 / (100.f), 0, 0, 1};
PID_TypeDef pidA = {1 / (M_PI / 2), 0, 0, 1};
//Distance to wall:L
float value_Len;
const float set_Len = 80.f;
//Angle theta of car:theta
/*float data_angle.z*/
float value_angle;
float value_angle0 = 0;
float set_angle = 0;
//Angle of servo
int16_t value_servo = 0;
int16_t value_servo0 = -73;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const *argument);
void StartTask_Ctrl(void const *argument);
void StartTaskMPU6050(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void Key_Loop(void);
void Cali_Alpha(void);
void Step_Run(void);
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
	osThreadDef(Task_Ctrl, StartTask_Ctrl, osPriorityIdle, 0, 128);
	Task_CtrlHandle = osThreadCreate(osThread(Task_Ctrl), NULL);

	/* definition and creation of Task_MPU6050 */
	osThreadDef(Task_MPU6050, StartTaskMPU6050, osPriorityIdle, 0, 128);
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

		//Ctrl
		if (Flag_Mode == FLAG_MODE_CALI_G)
		{
			printf("\x1b[1BCALI:*G%d*   \r\x1b[1A", Value_CaliGz);
		}
		else
		{
			printf("\x1b[1BANGLE:*G%-.1f*   \r\x1b[1A", data_angle.z * 57.2957795f);
		}
		if (TF_HR04)
		{
			TF_HR04 = 0;
			printf("\x1b[2BHR04:*D%dcm  \t%dcm  *  \r\x1b[2A",
				   HR04_GetIntCm(0), HR04_GetIntCm(1));
		}
		printf("\x1b[3BNear:*S%02X\t%02X\t%02X\t%02X\t*	\r\x1b[3A",
			   NearSor[0], NearSor[1], NearSor[2], NearSor[3]);
		printf("\x1b[4BPID:*P%.2f %.2f %.2f*    \r\x1b[4A",
			   pidL.Kp, pidL.Ki, pidL.Kd);
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
		if (Flag_Mode == FLAG_MODE_CALI_S)
			Cali_Alpha();
		else if (Flag_Mode == FLAG_MODE_RUN)
			Step_Run();

		TIM3->CCR1 = 1500 + (value_servo - value_servo0);

		Key_Loop();
		if (KEY_IS_DOWN(0))
		{
			//Start
			Flag_Mode = FLAG_MODE_RUN;
		}
		if (KEY_IS_DOWN(1))
		{
			//Inte Calibration Zero
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
		osDelay(100);
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
			MPU_GetIntDatas(&datas);

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
//Run
void Step_Run(void)
{
	float tmpF;
	//vA = A+A0
	value_angle = (data_angle.z + value_angle0);
	//Value L = D*cos(A+A0)
	value_Len = HR04_GetFloatCm(1) * cos(value_angle);
	//PID Length -> set_angle
	tmpF = PID_Process(&pidL, set_Len - value_Len);
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
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
