#include "Ano_Math.h"
#include "LineFollow.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"
#include "AutoTakeOff.h"
#include "AutoLand.h"

#define FlightSpeed 10  //宏定义，飞行速度
#define ANGLESPEED_KP 0.3f //PID比例系数
#define ANGLESPEED_KD 0.08f  //

struct LFAngleCalcuStruct
{
	float ErrorLastL;
	float ErrorLast;
	float Error;
	float OutputDelta;
	float Target_angle;
};
struct LFAngleCalcuStruct LFAngleCalcuPID;
float AngleSpeed = 0;

void LFAnglePID_Init(void);
void AngleSpeedCalcu(float realAngle);
void KeepAngle(float realAngle);

extern unsigned char LandProgramUnlock_sta;
extern unsigned char Is_AutoLand;

/**********************************************************************************************************
*函 数 名: Program_Ctrl_User_Set_YAWdps
*功能说明: 程控功能，航向速度设定（实时控制）
*参    数: 速度（度每秒，正为右转，负为左转）
*返 回 值: 无
**********************************************************************************************************/
//
/*
起飞前，飞机机头方向向右，设x轴速度起飞

*/
unsigned int myTimer = 0;
unsigned int NoflagTimer = 0;
unsigned char Task = 0;//飞行任务执行阶段:直行，左转，右转
unsigned int TurnCount = 0;//转向状态计数
unsigned int TCount = 0;//任务执行次数（判断是否检测到黑圆，进行降落），计算总共执行任务（直行，左转，右转）的次数，如果次数大于8说明飞机将要降落（）
unsigned char flightflag = 0;//k210标志位：
short int angle;
short int distance;
unsigned char crossflag;
short int crossX;
short int crossY;

void Line_Follow(unsigned int KeepHeight_cm,unsigned char dT_ms)
{
	flightflag = Can_K210.flag;
	angle = Can_K210.angle;
	distance = Can_K210.distance;
	crossflag = Can_K210.crossflag;
	crossX = Can_K210.x;
	crossY = Can_K210.y;
	switch(Task)
	{
		//任务状态1，直行
		//直行过程中如果任务数不够8，此时检测到了圆，继续直行（加一个if）
		//直行过程需要pid控制y轴，靠近直线正上方
		case 0:
		{
			Task += 1;
		}
		case 1:
		{
			//myTimer += dT_ms;
			if(myTimer < 4000)
			{
				pc_user.vel_cmps_set_h[0] = 5.f;
				myTimer += dT_ms;
			}else
			{
				if(flightflag == 0)//啥也没有检测到
				{
					NoflagTimer += dT_ms;////无标志状态计时置0
					
					if(NoflagTimer < 2000)//继续向前飞
					{
						pc_user.vel_cmps_set_h[0] = FlightSpeed -3.f;
					}
					else if(NoflagTimer < 5000)//超过3s,速度减半
					{
						pc_user.vel_cmps_set_h[0] = FlightSpeed - 3.f;
					}
	//				else//一个任务阶段内超过5s没有检测到，降落
	//				{
	//					NoflagTimer = 0;
	//					Task = 99;
	//				}
				}
				else if(flightflag == 1)//先直行，向前走
				{
					pc_user.vel_cmps_set_h[0] = FlightSpeed;
					
					NoflagTimer = 0;
				}
				else if(flightflag == 2)//如果检测到左转
				{
					NoflagTimer = 0;//无标志状态计时置0
					if(TurnCount < 5)//转向请求，计数五十次
					{
						TurnCount++;
					}
					else
					{
						pc_user.vel_cmps_set_h[0] = 0;//前进速度置0
						pc_user.vel_cmps_set_h[1] = 0;
						TurnCount = 0;//转向请求计数置0
						myTimer = 0;//阶段任务执行时间置0
						
						Task = 2;//调转到下一任务：左转
						TCount += 1;//任务数加1
					}
				}
			}
			
		}break;
		//任务状态2：左转任务
		case 2:
		{
			//逆时针旋转90度
			if(myTimer <= 1000){
				//pc_user.vel_cmps_set_h[0] = 0;//前进速度置0
				//pc_user.vel_cmps_set_h[1] = 0;
				myTimer += dT_ms;
			}
			else if (myTimer <= 2000)
			{		
				//6秒90度,给90的话会转过头
				Program_Ctrl_User_Set_YAWdps(-85);//速度每秒15度
				myTimer += dT_ms;
			}
			else if(myTimer <= 2200)//转向结束,自旋速度置0
			{
				Program_Ctrl_User_Set_YAWdps(0);
				
				myTimer += dT_ms;
			}
			else if(myTimer <= 3200)//转向结束,自旋速度置0
			{
				pc_user.vel_cmps_set_h[1] = 5;
				myTimer += dT_ms;
			}
			else if(myTimer <= 3300)//转向结束,自旋速度置0
			{
				pc_user.vel_cmps_set_h[1] = 0;
				myTimer += dT_ms;
			}
			else if(myTimer <= 6300)//左转完成，向前行进20cm,进入直线
			{
				pc_user.vel_cmps_set_h[0] = 10;//  速度：10cm/s 时间：2s
				myTimer += dT_ms;
			}
//			else if(myTimer <= 8200)//转向结束,自旋速度置0
//			{
//				Program_Ctrl_User_Set_YAWdps(0);
//				myTimer += dT_ms;
//			}
			else
			{
				//pc_user.vel_cmps_set_h[0] = 0;//前进速度置0
				//pc_user.vel_cmps_set_h[1] = 0;
				myTimer = 0;//阶段任务执行时间置0
				
				if(TCount < 8){
					Task = 1;
				}
				else{
					Task = 99;//调转到下一任务：继续直行
				}
				
				
				TCount += 1;//任务数加1
				
			}
			
		}break;
		//线状态   右转
		case 3:
		{
			
		}break;
		case 4:
		{
			
		}break;
		case 99://降落
		{
			LandProgramUnlock_sta = 1;
			Is_AutoLand = Yes;
		}break;
		
	};
}

/*
计算巡线过程中的角速度
参数：K210返回的角度
*/
void AngleSpeedCalcu(float realAngle)
{
	//LFAngleCalcuPID.Target_angle = 0;//目标角度为0
	LFAngleCalcuPID.ErrorLastL = LFAngleCalcuPID.ErrorLast;
	LFAngleCalcuPID.ErrorLast = LFAngleCalcuPID.Error;
	LFAngleCalcuPID.Error = 0 - realAngle;
	
	LFAngleCalcuPID.OutputDelta = ANGLESPEED_KP * (LFAngleCalcuPID.Error - LFAngleCalcuPID.ErrorLast)
	+ ANGLESPEED_KD * (LFAngleCalcuPID.Error - LFAngleCalcuPID.ErrorLast * 2.f 
	+ LFAngleCalcuPID.ErrorLastL);
	
	AngleSpeed += LFAngleCalcuPID.OutputDelta;
	AngleSpeed = LIMIT(AngleSpeed, -KEEPHEIGHTSPEEDMAX, KEEPHEIGHTSPEEDMAX);
};

/*
功能：飞行器直线行驶时控制角度为0
参数：K210识别到的直线相对与飞行器的角度
*/
void KeepAngle(float realAngle)
{
	AngleSpeedCalcu(realAngle);                     	//计算速度                                        
	Program_Ctrl_User_Set_YAWdps(AngleSpeed);  //使用程控赋值速度
}



void LFAnglePID_Init(void)
{
	LFAngleCalcuPID.Error = 0;
	LFAngleCalcuPID.ErrorLast = 0;
	LFAngleCalcuPID.ErrorLastL = 0;
	LFAngleCalcuPID.OutputDelta = 0;
	LFAngleCalcuPID.Target_angle = 0;
};
