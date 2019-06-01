/* #HR04 */

/* Inc */
#include "HR04.h"

/* Var */
#define HR04_PERIOD (1000000 * 2 / 340 * 4) - 1 //8Meter HR04
uint8_t TF_HR04 = 0;
uint16_t ICReadValue[2] = {0, 0};
uint16_t HR04_Value[2] = {0, 0};

/* Fun */
void HR04_Config(void)
{
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
	//Disable output
	TIM2->ARR = HR04_PERIOD;
	TIM2->CCR1 = HR04_PERIOD - 10;
	TIM2->CCR3 = HR04_PERIOD - 10;
	TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC3E);
	HAL_TIM_Base_Start_IT(&htim2);
	TIM2->DIER |= TIM_DIER_UDE | TIM_DIER_CC2IE | TIM_DIER_CC4IE;
	TIM2->CR1 |= TIM_CR1_CEN;
}

#define TIM_CCER_CC24P (TIM_CCER_CC2P | TIM_CCER_CC4P)
void HR04_IRQHandler(void)
{
	static uint8_t index = 0;
	uint16_t TIM_CCER_CCxE, TIM_SR_CCxIF;
	/* TIM2 Update interrupt */
	if (TIM2->SR & TIM_SR_UIF)
	{
		//Clear flag
		TIM2->SR &= ~TIM_SR_UIF;
		//Set CCxE (Output Enable)
		TIM_CCER_CCxE = (index == 0) ? TIM_CCER_CC1E : TIM_CCER_CC3E;
		if ((TIM2->CCER & TIM_CCER_CCxE) == 0)
		{
			//If Output is disable,It means that Timeout
			HR04_Value[index] = HR04_PERIOD - 1;
			GPIOA->BSRR = (index == 0) ? GPIO_BSRR_BS7 : GPIO_BSRR_BR7;
			//Next index.
			index ^= 0x01;
		}
		//Disable output and Enable output if timeout
		TIM_CCER_CCxE = (index == 0) ? TIM_CCER_CC1E : TIM_CCER_CC3E;
		TIM2->CCER ^= TIM_CCER_CCxE;
	}
	/* TIM2 Capture 2 interrupt */
	if (TIM2->SR & (TIM_SR_CC2IF | TIM_SR_CC4IF))
	{
		TIM_SR_CCxIF =
			TIM2->SR & ((index == 0) ? TIM_SR_CC2IF : TIM_SR_CC4IF);
		//Clear flag
		TIM2->SR &= ~(TIM_SR_CC2IF | TIM_SR_CC4IF);
		if (TIM_SR_CCxIF)
		{
			if (TIM2->CCER & TIM_CCER_CC24P)
			{
				/* Get the Input Capture value */
				HR04_Value[index] = (index == 0) ? TIM2->CCR2 : TIM2->CCR4;

				/* Capture computation */
				if (HR04_Value[index] > ICReadValue[index])
				{
					HR04_Value[index] = (HR04_Value[index] - ICReadValue[index]);
				}
				else
				{
					HR04_Value[index] = ((HR04_PERIOD - ICReadValue[index]) + HR04_Value[index]);
				}
				TIM2->CCER &= ~TIM_CCER_CC24P;
				//Next index
				index ^= 0x01;
				//Enable output
				TIM_CCER_CCxE = (index == 0) ? TIM_CCER_CC1E : TIM_CCER_CC3E;
				TIM2->CCER |= TIM_CCER_CCxE;
				//Reset Count
				TIM2->CNT = 0;
				TF_HR04 = 1;
			}
			else
			{
				/* Get the Input Capture value */
				ICReadValue[index] = (index == 0) ? TIM2->CCR2 : TIM2->CCR4;
				TIM2->CCER |= TIM_CCER_CC24P;
			}
		}
	}
}
