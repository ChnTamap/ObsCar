/* MPU6050 Define */
#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "i2c.h"

#define ADDR_MPU6050_WRITE 0xD0
#define ADDR_MPU6050_READ (ADDR_MPU6050_WRITE | 0x01)
#define REG_SMPLRT_DIV 0x19   //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define REG_CONFIG 0x1A		  //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define REG_GYRO_CONFIG 0x1B  //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define REG_ACCEL_CONFIG 0x1C //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_YOUT_L 0x46
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_PWR_MGMT_1 0x6B //��Դ����������ֵ��0x00(��������)
#define REG_WHO_AM_I 0x75   //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

typedef struct
{
	float x;
	float y;
	float z;
} MPU_V3D;
typedef struct
{
	float x;
	float y;
	float z;
	float w;
} MPU_Quat;
typedef struct
{
	int16_t Ax;
	int16_t Ay;
	int16_t Az;
	int16_t Gx;
	int16_t Gy;
	int16_t Gz;
} MPU_Datas;

#define MPU_Kp 0.125f
#define MPU_Ki 0.f

HAL_StatusTypeDef MPU_Init(uint8_t timeout);
HAL_StatusTypeDef MPU_WriteByte(uint8_t addr, uint8_t value);
HAL_StatusTypeDef MPU_Read(uint8_t addr, uint8_t *value, uint8_t len);

void MPU_TurnWordHL(uint16_t *buf);
HAL_StatusTypeDef MPU_GetIntDatas(MPU_Datas *datas);
void MPU_GetFloatDatas(MPU_V3D *vA, MPU_V3D *vG, MPU_Datas *datas);

void MPU_QuatMulV3d(MPU_V3D *vo, MPU_Quat *q, MPU_V3D *v);
void MPU_NormVector(MPU_V3D *v);
void MPU_NormQuat(MPU_Quat *q);

/* Get Quat from Gyro integral and Accel*/
void MPU_Filter(MPU_V3D *vG, MPU_V3D *vA, MPU_V3D *eInt, MPU_Quat *q, float halfT);
void MPU_GetAngle(MPU_Quat *q, float *roll, float *pitch, float *yaw);
void MPU_GetAccelBase(MPU_Quat *baseQ, MPU_V3D *acc);

#endif
