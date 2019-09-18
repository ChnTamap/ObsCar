#include "MPU6050.h"
#define M_PI 3.1415926535897932f
#include "math.h"

void MPU_OutBusy(void)
{
	/*GPIO_InitTypeDef GPIO_InitStruct;
	 HAL_I2C_MspDeInit(&hi2c1);

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	while (1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_8;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			HAL_I2C_MspInit(&hi2c1);
			MX_I2C1_Init();

			break;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	} */
	if (I2C1->SR2 & I2C_SR2_BUSY)
	{
		I2C1->CR1 |= I2C_CR1_SWRST;
	}
	if (I2C1->SR1 & I2C_SR1_TIMEOUT)
	{
		I2C1->CR1 &= I2C_CR1_PE;
		I2C1->CR1 |= I2C_CR1_PE;
		I2C1->CR1 |= I2C_CR1_SWRST;
	}
}

/* 
ACC_CFG	GYR_CFG
00	2g	00	250
01	4g	01	500
10	8g	10	1K
11	16g	11	2K
 */
HAL_StatusTypeDef MPU_Init(uint8_t timeout)
{
	HAL_StatusTypeDef sta;
	while (timeout--)
	{
		sta = HAL_OK;
		sta += MPU_WriteByte(REG_PWR_MGMT_1, 0x00);
		sta += MPU_WriteByte(REG_SMPLRT_DIV, 0x07);
		sta += MPU_WriteByte(REG_CONFIG, 0x06);
		sta += MPU_WriteByte(REG_ACCEL_CONFIG, 0x00);
		sta += MPU_WriteByte(REG_GYRO_CONFIG, 0x00);
		if (sta == HAL_OK)
			return HAL_OK;
		MPU_OutBusy();
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef MPU_WriteByte(uint8_t addr, uint8_t value)
{
	return HAL_I2C_Mem_Write(&hi2c1, ADDR_MPU6050_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10);
}

HAL_StatusTypeDef MPU_Read(uint8_t addr, uint8_t *value, uint8_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, ADDR_MPU6050_READ, addr, I2C_MEMADD_SIZE_8BIT, value, len, 0x10);
}

void MPU_TurnWordHL(uint16_t *buf)
{
	uint8_t tmp;
	tmp = *((uint8_t *)buf);
	*((uint8_t *)buf) = *(((uint8_t *)buf) + 1);
	*(((uint8_t *)buf) + 1) = tmp;
}

HAL_StatusTypeDef MPU_GetIntDatas(MPU_Datas *datas)
{
	uint32_t i;
	HAL_StatusTypeDef sta;
	MPU_OutBusy();
	MPU_Read(REG_ACCEL_XOUT_H, (uint8_t *)(datas), 6);
	sta = MPU_Read(REG_GYRO_XOUT_H, (uint8_t *)(datas) + 6, 6);
	for (i = 0; i < 6; i++)
	{
		MPU_TurnWordHL((uint16_t *)datas + i);
	}
	return sta;
}

#define MPU_ACC_LSB (2.f / 32768)				  //LSB/g
#define MPU_GYRO_LSB (250.f * M_PI / 32768 / 180) //LSB/r/s
void MPU_GetFloatDatas(MPU_V3D *vA, MPU_V3D *vG, MPU_Datas *datas)
{
	//Acc
	vA->x = datas->Ax;
	vA->y = datas->Ay;
	vA->z = datas->Az;
	vA->x *= MPU_ACC_LSB;
	vA->y *= MPU_ACC_LSB;
	vA->z *= MPU_ACC_LSB;
	//Gyro
	vG->x = datas->Gx;
	vG->y = datas->Gy;
	vG->z = datas->Gz;
	vG->x *= MPU_GYRO_LSB;
	vG->y *= MPU_GYRO_LSB;
	vG->z *= MPU_GYRO_LSB;
}

void MPU_NormQuat(MPU_Quat *q)
{
	float q0 = q->x;
	float q1 = q->y;
	float q2 = q->z;
	float q3 = q->w;
	float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (norm == 0)
	{
		q->x = 0;
		q->y = 0;
		q->z = 0;
		q->w = 1;
	}
	else
	{
		norm = 1.f / norm;
		q->x *= norm;
		q->y *= norm;
		q->z *= norm;
		q->w *= norm;
	}
}

void MPU_NormVector(MPU_V3D *v)
{
	float vx = v->x;
	float vy = v->y;
	float vz = v->z;
	float norm = sqrtf(vx * vx + vy * vy + vz * vz);
	if (norm == 0)
	{
		v->x = 0;
		v->y = 1;
		v->z = 0;
	}
	else
	{
		v->x *= norm;
		v->y *= norm;
		v->z *= norm;
	}
}

void MPU_GetAccelBase(MPU_Quat *baseQ, MPU_V3D *acc)
{
	float G;
	MPU_NormVector(acc);
	G = sqrtf(0.5f * (1.f + acc->z));
	baseQ->x = G * acc->y;
	baseQ->y = -G * acc->x;
	baseQ->z = 0;
	baseQ->w = sqrtf((1.f + acc->z) / 2.f);
	MPU_NormQuat(baseQ);
}

void MPU_QuatMulV3d(MPU_V3D *vo, MPU_Quat *q, MPU_V3D *v)
{
	float q0, q1, q2, q3;
	float vx, vy, vz;
	q0 = q->x;
	q1 = q->y;
	q2 = q->z;
	q3 = q->w;
	vx = v->x;
	vy = v->y;
	vz = v->z;
	vo->x = vx + vy * q2 * q3 - vz * q1 * q3;
	vo->y = vy + vz * q0 * q3 - vx * q2 * q3;
	vo->z = vz + vx * q1 * q3 - vy * q0 * q3;
}

void MPU_Filter(MPU_V3D *vG, MPU_V3D *vA, MPU_V3D *eInt, MPU_Quat *q, float halfT)
{
	float q0, q1, q2, q3, gx, gy, gz;
	// float ex, ey, ez;
	q0 = q->x;
	q1 = q->y;
	q2 = q->z;
	q3 = q->w;

	MPU_NormVector(vA);
	/* 
	gx = 2.f * (q1 * q3 - q0 * q2);
	gy = 2.f * (q0 * q1 + q2 * q3);
	gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	ex = (vA->y * gz - vA->z * gy);
	ey = (vA->z * gx - vA->x * gz);
	ez = (vA->x * gy - vA->y * gx);
	eInt->x += ex;
	eInt->x += ey;
	eInt->x += ez;
	vG->x += MPU_Kp * ex + eInt->x * MPU_Ki;
	vG->y += MPU_Kp * ey + eInt->y * MPU_Ki;
	vG->z += MPU_Kp * ez + eInt->z * MPU_Ki; 
	 */
	gx = vG->x;
	gy = vG->y;
	gz = vG->z;
	q->x += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q->y += (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q->z += (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q->w += (q0 * gz + q1 * gy - q2 * gx) * halfT;
	MPU_NormQuat(q);
}

void MPU_GetAngle(MPU_Quat *q, float *roll, float *pitch, float *yaw)
{
	*roll = asin(2.f * (q->x * q->y + q->z * q->w));  // * 57.2957795f;
	*pitch = asin(2.f * (q->z * q->x + q->y * q->w)); // * 57.2957795f;
	*yaw = asin(2.f * (q->y * q->z + q->x * q->w));   // * 57.2957795f;
}
