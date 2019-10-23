#include "tim.h"
#include "MPU6050.h"
#include "HR04.h"
#include "PID.h"
#include "math.h"
#include "log.h"

//#Math
#define M_PI 3.14159265358f
//#Para
#define Para_Len 7
uint16_t Para_List[Para_Len] = {8, 95, 18, 17, 10, 100, 70};
#define Para_ANGLE_READY Para_List[0]
#define Para_OBS_DIS Para_List[1]
#define Para_TURN_ANGLE Para_List[2]
#define Para_TURN_BACK Para_List[3]
#define Para_TURN_MAX Para_List[4]
#define Para_DIS_DIFF Para_List[5]
#define Para_RUN_LEN Para_List[6]
//#Data
uint8_t TF_READY = 0xFF;
uint8_t TF_MPU6050 = 0;
enum
{
	FLAG_MODE_CLOSE = 0,
	FLAG_MODE_CALI_G, //静态校零陀螺仪Gyro
	FLAG_MODE_READY,  //准备滑行
	FLAG_MODE_CALI_S, //动态校零舵机value_Servo
	FLAG_MODE_CALI_T, //动态修正绝对角度Theta
	FLAG_MODE_RUN,	//稳定滑行阶段
	FLAG_MODE_OBS	 //避障阶段
};
uint8_t Flag_Mode = 0;
HAL_StatusTypeDef Flag_I2C = HAL_OK;
//避障阶段
uint8_t Flag_Step = 0;
int32_t Value_CaliGx, Value_CaliGy, Value_CaliGz;
uint16_t Value_CaliT = 0;
uint16_t EncodeSor[2] = {0, 0};
uint8_t NearSor[4];
#define NearSorL1 NearSor[1]
#define NearSorL2 NearSor[0]
#define NearSorR1 NearSor[2]
#define NearSorR2 NearSor[3]
#define NS_IS_DOWN(v) (v == 0x80)
#define NS_IS_PRESS(v) (v == 0x00)
#define NS_IS_UP(v) (v == 0x7F)
#define NS_IS_RES(v) (v == 0xFF)
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
// const float set_Len = 60.f;
#define set_Len ((float)Para_RUN_LEN)
//Angle theta of car:theta
/*float data_angle.z*/
float value_angle;
float value_angle0 = 0;
float set_angle = 0;
//Angle of servo
int16_t value_servo = 0;
int16_t value_servo0 = -73;
//#Obs
//TURN_ANGLE Servo
#define TURN_ANGLE (-Para_TURN_ANGLE * 1000 / 90)
#define TURN_BACK (-Para_TURN_BACK * 1000 / 90)
#define DIS_DIFF Para_DIS_DIFF
uint8_t Flag_Dir = 1;
int16_t Flag_Angle;
uint16_t value_pDis;
uint8_t *NearSorX1;
uint8_t *NearSorX2;
//#GUI
uint8_t GUI_Mode = 0;
uint8_t GUI_Sele = 0;
