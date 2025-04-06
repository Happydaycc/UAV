#include "Ano_Math.h"
#include "Ano_RC.h"
#include "Ano_ProgramCtrl_User.h"
#include "AutoTakeOff.h"
#include "Ano_FlightCtrl.h"
#include "Can_T265.h"

#define TAKEOFFSPEED_KP 0.3f  //起飞PID比例系数
#define TAKEOFFSPEED_KD 0.15f //微分系数
#define KEEPHEIGHTSPEED_KP 0.45f //定高PID比例系数
#define KEEPHEIGHTSPEED_KD 0.15f  //

extern _pc_user_st pc_user;

unsigned char Is_AutoTakeOff = 0;
unsigned char KHFlag = 0;
unsigned char OffFlag = 0;
unsigned char OffProgramUnlock_sta = 0;

struct AutoTakeOffSpeedCalcuStruct
{
	float ErrorLastL;
	float Errorlast;
	float Error;
	float OutputDelta;
	//float AutoTakeOffSpeed;
};
struct AutoTakeOffSpeedCalcuStruct AutoTakeOffSpeedCalcuPID; 
float AutoTakeOffSpeed = 0;
struct KeepHeightSpeedCalcuStruct
{
	float ErrorLastL;
	float ErrorLast;
	float Error;
	float OutputDelta;
	float KeepHeight_TargetHeight_cm;
};

//定义变量

float KeepHeightSpeed = 0;
struct KeepHeightSpeedCalcuStruct	KeepHeightSpeedCalcuPID;
void AutoTakeOffSpeedCalcu(float TargetHeight_cm);
void KeepHeightSpeedCalcu(unsigned int Height);
void PID_Init(void);
//修改：添加了注释 2023.11.29 报错：Undefined symbol 
extern unsigned char LandProgramUnlock_sta;
extern unsigned char Is_AutoLand;
extern unsigned char Basic1Flag;
extern unsigned char Advanced2Flag;
unsigned char TaskChoose;
unsigned int RC_Timer_ms = 0;
unsigned char CHFlag;

void Remote_Control(unsigned char dT_ms)
{
	//if(CH_N[6]){}通过debug查看遥控器拨杆与CH_N的对应关系，经过查看此时使用的遥控器SWC对应飞控的CH_N[6]
	if (CH_N[5] < -300)
	{
		OffProgramUnlock_sta = 0; // ??0±íê?￡?ò??ü?e·é3ìDò?÷ì?±???×? 
		Is_AutoTakeOff = No; // ??No±íê?￡?2?×?±?ò??ü?e·é
		LandProgramUnlock_sta = 0;
		Is_AutoLand = No; 
	}
	else if (CH_N[5] < 200)
	{
		OffProgramUnlock_sta = 1; 
		Is_AutoTakeOff = Yes; 	
	}
	else 
	{	
		LandProgramUnlock_sta = 1;
		Is_AutoLand = Yes; 
	}
	switch(CHFlag){
		case 0:{
//			if (CH_N[6] < -300 &&CH_N[7] < -300){ //00任务1
//				TaskChoose = 1;
//			}
//			else if(CH_N[6] < -300&&CH_N[7] >300){ //01任务2
//				TaskChoose = 2;
//			}
//			else if(CH_N[6] > 300&&CH_N[7] > 300){  //10任务3
//				TaskChoose = 3;
//			}
			if (CH_N[6] < -300){ //00任务1
				TaskChoose = 1;
			}
			else if(CH_N[6] > 300){ //01任务2
				TaskChoose = 2;
			}
			
			CHFlag = 2;
		}break;
		case 1:
		break;
	}	
}
/*
函数名：AutoTakeOff
功能说明:一键起飞任务
参数：周期时间
*/
unsigned int Timer = 0;

unsigned char AutoTakeOff_ConfirmCount = 0;
void AutoTakeOff(unsigned char dT_ms)
{

	if (flag.unlock_sta == 0) // flag.unlock_sta?a??￡??′?a??￡?·é?úè??a??￡??ò?-?-￡?
	{
		flag.taking_off = 0;
		flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;
		PID_Init();
	}
	else
	{
		if (OffProgramUnlock_sta == 1) // ?úí¨μà?D????·é?úò??ü?e·é?￡
		{
			if (Timer <= 2000)
			{		
				Timer += dT_ms; 
			}
			
			if (Timer > 2000) // 2soó?e·é
			{
				Timer += dT_ms;
								
				if (Is_AutoTakeOff == Yes && KHFlag == 0) // Is_AutoTakeOff?a??￡??′?úò￡???÷í¨μà?Dò??3?μ￡?·éê?òa?óò??ü?e·é￡??ò?-?-
				{
					flag.taking_off = 1;
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					AutoTakeOffSpeedCalcu(OFF_CM);
					Program_Ctrl_User_Set_Zcmps(AutoTakeOffSpeed);
					T265_xy_axis(0,0);
				}
				
				
				if (ABS(OFF_CM - RELATIVE_HEIGHT_CM) <= 5)
				{
					if (AutoTakeOff_ConfirmCount <= 5)
					{
						AutoTakeOff_ConfirmCount ++;
					}
					else
					{
						AutoTakeOff_ConfirmCount = 0;
						KHFlag = 1;
					}					
					//flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;											
				}
			
				if (Timer > (2000 + 1400)) // 起飞1.4秒后，若摇杆控制，则退出一键起飞
				{
					Timer = 3500;
					if (ABS(fs.speed_set_h_norm[Z]) > 0.1f) // 如果飞手通过摇杆控制飞机，则退出一键起飞程序。
					{
						flag.taking_off = 0;
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						return ;
					}
				}
 		}

	}
}
};
	



/*
计算起飞时的速度
参数：目标高度，速度
*/

void AutoTakeOffSpeedCalcu(float TargetHeight_cm)
{

	AutoTakeOffSpeedCalcuPID.Errorlast = AutoTakeOffSpeedCalcuPID.Error;
	AutoTakeOffSpeedCalcuPID.Error = TargetHeight_cm - RELATIVE_HEIGHT_CM;
	
	AutoTakeOffSpeed = TAKEOFFSPEED_KP*AutoTakeOffSpeedCalcuPID.Error+
	TAKEOFFSPEED_KD *(AutoTakeOffSpeedCalcuPID.Error-AutoTakeOffSpeedCalcuPID.Errorlast);
	
	AutoTakeOffSpeed = LIMIT(AutoTakeOffSpeed, -AUTOTAKEOFFSPEEDMAX, AUTOTAKEOFFSPEEDMAX);

};


/*
功能：飞行器定高
参数：定高所设高度
*/
void KeepHeight(unsigned int Height)
{

	KeepHeightSpeedCalcu(Height);                     	//计算速度                                        
	Program_Ctrl_User_Set_Zcmps(KeepHeightSpeed);  //使用程控赋值速度
	
}



/*
计算定高过程中的速度
参数：目标高度
*/
void KeepHeightSpeedCalcu(unsigned int Height)
{
	KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm = Height;
	KeepHeightSpeedCalcuPID.ErrorLast = KeepHeightSpeedCalcuPID.Error;
	KeepHeightSpeedCalcuPID.Error = KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm - RELATIVE_HEIGHT_CM;
	
	KeepHeightSpeed =  KEEPHEIGHTSPEED_KP*KeepHeightSpeedCalcuPID.Error+KEEPHEIGHTSPEED_KD *
	(KeepHeightSpeedCalcuPID.Error-KeepHeightSpeedCalcuPID.ErrorLast);
	
	KeepHeightSpeed = LIMIT(KeepHeightSpeed, -KEEPHEIGHTSPEEDMAX, KEEPHEIGHTSPEEDMAX);

};


//起飞和定高PID结构体初始化
void PID_Init(void)
{
	AutoTakeOffSpeedCalcuPID.Error = 0;
	AutoTakeOffSpeedCalcuPID.Errorlast = 0;
	AutoTakeOffSpeedCalcuPID.ErrorLastL = 0;
	AutoTakeOffSpeedCalcuPID.OutputDelta = 0;
	
	KeepHeightSpeedCalcuPID.Error = 0;
	KeepHeightSpeedCalcuPID.ErrorLast = 0;
	KeepHeightSpeedCalcuPID.ErrorLastL = 0;
	KeepHeightSpeedCalcuPID.OutputDelta = 0;
	KeepHeightSpeedCalcuPID.KeepHeight_TargetHeight_cm = 0;
	KeepHeightSpeed = 0;
};
