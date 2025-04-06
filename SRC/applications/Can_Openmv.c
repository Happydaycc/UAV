#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Ano_ProgramCtrl_User.h"

#define	OPMV_Ang_KP 0.2f
#define	OPMV_Ang_KI 0.f
#define	OPMV_Ang_KD 0.1f

struct OPMV_Value
{ 
	float Ang_error;
	float Ang_errorLast;
	float Ang_errorLastL;
	float Ang_integral;
	float Ang_integralLast;
	float Ang_outputDelta;
	float Ang_output;
};

struct OPMV_Value OPMV;

void OPMV_PID_Init(void)
{
	OPMV.Ang_error = 0.f;
	OPMV.Ang_errorLast = 0.f;
	OPMV.Ang_errorLastL = 0.f;
	OPMV.Ang_integral = 0.f;
	OPMV.Ang_integralLast = 0.f;
	OPMV.Ang_output = 0.f;
	OPMV.Ang_outputDelta = 0.f; 
}

char OPMV_x_flag = 1;
short int OPMV_LastAng = 0;
float OPMV_Angle_PID_calculation(short int Target_Angle, short int Real_Ang)//得到PIDx速度
{	
	OPMV_LastAng = Real_Ang;
			
	if (OPMV_x_flag == 1)
	{
		OPMV_PID_Init();
		OPMV_x_flag = 0;
	}
	else
	{
		
		OPMV.Ang_outputDelta = OPMV_Ang_KP * (OPMV.Ang_error - OPMV.Ang_errorLast)+
		OPMV_Ang_KI * (OPMV.Ang_integral - OPMV.Ang_integralLast) +
		OPMV_Ang_KD * (OPMV.Ang_error - 2 * OPMV.Ang_errorLast + OPMV.Ang_errorLastL);

		OPMV.Ang_errorLastL = OPMV.Ang_errorLast;
		OPMV.Ang_errorLast = OPMV.Ang_error;

		OPMV.Ang_output += OPMV.Ang_outputDelta;
		OPMV.Ang_error = Target_Angle - Real_Ang; 
		OPMV.Ang_integralLast = OPMV.Ang_integral;
		OPMV.Ang_integral += OPMV.Ang_error;
		
	}		
	return -OPMV.Ang_output;				
}

float OPMV_Ang_speed;
void OPMV_Angle_freeze(short int Target_Angle, short int Real_Ang) 
{
	OPMV_Ang_speed = OPMV_Angle_PID_calculation(Target_Angle, Real_Ang);
	
	if (ABS(OPMV_Ang_speed) <= 0.5f)
	{
		OPMV_Ang_speed = 0;
	}
	
	OPMV_Ang_speed = LIMIT(OPMV_Ang_speed, -30, 30);
	pc_user.pal_dps_set = OPMV_Ang_speed ;			
}




