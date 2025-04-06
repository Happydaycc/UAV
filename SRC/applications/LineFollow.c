#include "Ano_Math.h"
#include "LineFollow.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"
#include "AutoTakeOff.h"
#include "AutoLand.h"

#define FlightSpeed 10  //�궨�壬�����ٶ�
#define ANGLESPEED_KP 0.3f //PID����ϵ��
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
*�� �� ��: Program_Ctrl_User_Set_YAWdps
*����˵��: �̿ع��ܣ������ٶ��趨��ʵʱ���ƣ�
*��    ��: �ٶȣ���ÿ�룬��Ϊ��ת����Ϊ��ת��
*�� �� ֵ: ��
**********************************************************************************************************/
//
/*
���ǰ���ɻ���ͷ�������ң���x���ٶ����

*/
unsigned int myTimer = 0;
unsigned int NoflagTimer = 0;
unsigned char Task = 0;//��������ִ�н׶�:ֱ�У���ת����ת
unsigned int TurnCount = 0;//ת��״̬����
unsigned int TCount = 0;//����ִ�д������ж��Ƿ��⵽��Բ�����н��䣩�������ܹ�ִ������ֱ�У���ת����ת���Ĵ����������������8˵���ɻ���Ҫ���䣨��
unsigned char flightflag = 0;//k210��־λ��
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
		//����״̬1��ֱ��
		//ֱ�й������������������8����ʱ��⵽��Բ������ֱ�У���һ��if��
		//ֱ�й�����Ҫpid����y�ᣬ����ֱ�����Ϸ�
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
				if(flightflag == 0)//ɶҲû�м�⵽
				{
					NoflagTimer += dT_ms;////�ޱ�־״̬��ʱ��0
					
					if(NoflagTimer < 2000)//������ǰ��
					{
						pc_user.vel_cmps_set_h[0] = FlightSpeed -3.f;
					}
					else if(NoflagTimer < 5000)//����3s,�ٶȼ���
					{
						pc_user.vel_cmps_set_h[0] = FlightSpeed - 3.f;
					}
	//				else//һ������׶��ڳ���5sû�м�⵽������
	//				{
	//					NoflagTimer = 0;
	//					Task = 99;
	//				}
				}
				else if(flightflag == 1)//��ֱ�У���ǰ��
				{
					pc_user.vel_cmps_set_h[0] = FlightSpeed;
					
					NoflagTimer = 0;
				}
				else if(flightflag == 2)//�����⵽��ת
				{
					NoflagTimer = 0;//�ޱ�־״̬��ʱ��0
					if(TurnCount < 5)//ת�����󣬼�����ʮ��
					{
						TurnCount++;
					}
					else
					{
						pc_user.vel_cmps_set_h[0] = 0;//ǰ���ٶ���0
						pc_user.vel_cmps_set_h[1] = 0;
						TurnCount = 0;//ת�����������0
						myTimer = 0;//�׶�����ִ��ʱ����0
						
						Task = 2;//��ת����һ������ת
						TCount += 1;//��������1
					}
				}
			}
			
		}break;
		//����״̬2����ת����
		case 2:
		{
			//��ʱ����ת90��
			if(myTimer <= 1000){
				//pc_user.vel_cmps_set_h[0] = 0;//ǰ���ٶ���0
				//pc_user.vel_cmps_set_h[1] = 0;
				myTimer += dT_ms;
			}
			else if (myTimer <= 2000)
			{		
				//6��90��,��90�Ļ���ת��ͷ
				Program_Ctrl_User_Set_YAWdps(-85);//�ٶ�ÿ��15��
				myTimer += dT_ms;
			}
			else if(myTimer <= 2200)//ת�����,�����ٶ���0
			{
				Program_Ctrl_User_Set_YAWdps(0);
				
				myTimer += dT_ms;
			}
			else if(myTimer <= 3200)//ת�����,�����ٶ���0
			{
				pc_user.vel_cmps_set_h[1] = 5;
				myTimer += dT_ms;
			}
			else if(myTimer <= 3300)//ת�����,�����ٶ���0
			{
				pc_user.vel_cmps_set_h[1] = 0;
				myTimer += dT_ms;
			}
			else if(myTimer <= 6300)//��ת��ɣ���ǰ�н�20cm,����ֱ��
			{
				pc_user.vel_cmps_set_h[0] = 10;//  �ٶȣ�10cm/s ʱ�䣺2s
				myTimer += dT_ms;
			}
//			else if(myTimer <= 8200)//ת�����,�����ٶ���0
//			{
//				Program_Ctrl_User_Set_YAWdps(0);
//				myTimer += dT_ms;
//			}
			else
			{
				//pc_user.vel_cmps_set_h[0] = 0;//ǰ���ٶ���0
				//pc_user.vel_cmps_set_h[1] = 0;
				myTimer = 0;//�׶�����ִ��ʱ����0
				
				if(TCount < 8){
					Task = 1;
				}
				else{
					Task = 99;//��ת����һ���񣺼���ֱ��
				}
				
				
				TCount += 1;//��������1
				
			}
			
		}break;
		//��״̬   ��ת
		case 3:
		{
			
		}break;
		case 4:
		{
			
		}break;
		case 99://����
		{
			LandProgramUnlock_sta = 1;
			Is_AutoLand = Yes;
		}break;
		
	};
}

/*
����Ѳ�߹����еĽ��ٶ�
������K210���صĽǶ�
*/
void AngleSpeedCalcu(float realAngle)
{
	//LFAngleCalcuPID.Target_angle = 0;//Ŀ��Ƕ�Ϊ0
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
���ܣ�������ֱ����ʻʱ���ƽǶ�Ϊ0
������K210ʶ�𵽵�ֱ�������������ĽǶ�
*/
void KeepAngle(float realAngle)
{
	AngleSpeedCalcu(realAngle);                     	//�����ٶ�                                        
	Program_Ctrl_User_Set_YAWdps(AngleSpeed);  //ʹ�ó̿ظ�ֵ�ٶ�
}



void LFAnglePID_Init(void)
{
	LFAngleCalcuPID.Error = 0;
	LFAngleCalcuPID.ErrorLast = 0;
	LFAngleCalcuPID.ErrorLastL = 0;
	LFAngleCalcuPID.OutputDelta = 0;
	LFAngleCalcuPID.Target_angle = 0;
};
