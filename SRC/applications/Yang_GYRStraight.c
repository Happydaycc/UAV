#include "Ano_FcData.h"
#include "Ano_FlightCtrl.h"
#include "Ano_Math.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_RC.h"
#include "Drv_OpenMV.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Imu.h"

#include "FlightCtrlTask.h"
#include "AutoTakeOff.h"
#include "AutoLand.h"

#define	GYR_KP 5.f
#define	GYR_KI 0.0f 
#define	GYR_KD 0.f

extern _pc_user_st pc_user;
struct GYR_Value
{
	float Error;
	float ErrorLast;
	float ErrorLastL;
	float Integral;
	float IntegralLast;
	float OutputDelta;
	float Output;
};
struct GYR_Value GYR; 

/*
��������GYRPID_Init
����˵����������Ѳֱ�����PID������ʼ��
������Ŀ��GYR.val[0]
����ֵ����
���ڣ�2022��7��17��
*/
void GYRPID_Init(void)
{
	GYR.Output = 0.f;	
	GYR.OutputDelta = 0.f;
	GYR.Error = 0.f;
	GYR.ErrorLast = 0.f;
	GYR.ErrorLastL = 0.f; 
	GYR.Integral = 0.f;
	GYR.IntegralLast = 0.f;
} 

/*
��������GYR_ReadInitAngle
����˵������ȡ�ɻ���ǰ��ͷ����Ƕ�
������������ָ��Angle
����ֵ����
���ڣ�2022��7��25��
*/
unsigned char GYR_ReadInitAngleOnlyFlag = 0;
unsigned int GYR_ReadInitAngleTimer = 0;
unsigned int GYRInitTimer = 0;
unsigned char GYR_ReadInitAngle(float *TargetAngle, unsigned char dT_ms)
{	
	if (GYR_ReadInitAngleOnlyFlag == 0)
	{
		if (GYR_ReadInitAngleTimer <= 1000)
		{
			GYR_ReadInitAngleTimer += dT_ms;
		}// �����Ǹ��ϵ����У���ʼֵ���ȶ�������Ϊ���ų���һ����
		else
		{
			GYR_ReadInitAngleTimer = 0;
			*TargetAngle = imu_data.yaw + 4; // �൱�ڳ�ʼ��GYR.val[0] 
			GYR_ReadInitAngleOnlyFlag = 1;			
		}		
	}
	
	return GYR_ReadInitAngleOnlyFlag;

}
/*
��������GYRPID_Calculation
����˵����������Ѳֱ�����PID��������
������GYR.val[0]
����ֵ��GYR.Output
���ڣ�2022��7��17��
*/
char GYROrdinalNum = 0;
float GYRPID_CalculationTargetAngle = 0.f;
float GYRPID_Calculation(float Angle)
{		
	switch(GYROrdinalNum)
	{
		case 0:
		{
			GYRPID_Init();	
			GYROrdinalNum = 1;
		}break;
		case 1:
		{

			GYR.OutputDelta = GYR_KP * (GYR.Error - GYR.ErrorLast)+
			GYR_KI * (GYR.Integral - GYR.IntegralLast)+
			GYR_KD * (GYR.Error - 2 * GYR.ErrorLast+GYR.ErrorLastL);
			
			GYR.ErrorLastL = GYR.ErrorLast;
			GYR.ErrorLast = GYR.Error;
			
			GYR.Output += GYR.OutputDelta;
			
			if ((GYRPID_CalculationTargetAngle <= 180 && GYRPID_CalculationTargetAngle >= 160)&& (imu_data.yaw >= -180 && imu_data.yaw <= -160))
			{
				GYR.Error = GYRPID_CalculationTargetAngle - imu_data.yaw - 360.f;
			}
			else if ((imu_data.yaw <= 180 && imu_data.yaw >= 160) && (GYRPID_CalculationTargetAngle >= -180 && 	GYRPID_CalculationTargetAngle <= -160.f))
			{
				GYR.Error = GYRPID_CalculationTargetAngle - imu_data.yaw + 360.f;
			}
			else 
			{
				GYR.Error = GYRPID_CalculationTargetAngle - imu_data.yaw;
			}
						
			GYR.IntegralLast = GYR.Integral;
			GYR.Integral += GYR.Error;					
		}break; 
	}
	
	return GYR.Output;
} 

/*
��������GYR_Straight
����˵����������Ѳֱ�����PID��������
������GYR.val[0] -> ��������ֵ, StateNum -> ��0��ʾû����ɣ���1��ʾ�Ѿ����
����ֵ��GYR.Output
���ڣ�2022��7��17��
*/
float GYR_wSpeed = 0;
void GYR_Straight(void)
{	
	GYR_wSpeed = GYRPID_Calculation(imu_data.yaw);

	if (ABS(GYR_wSpeed) <= 0.5f) // ���wSpeedС��0.5����Ϊ�����㡣
	{
		GYR_wSpeed = 0;
	}

	GYR_wSpeed = LIMIT(GYR_wSpeed, -40, 40);
	
	if (ABS(GYR_wSpeed) >= 4)
	{
		pc_user.pal_dps_set = GYR_wSpeed;	
	}
			
}



