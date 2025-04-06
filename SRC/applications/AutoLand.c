#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightCtrl.h"

#include "FlightCtrlTask.h"
#include "AutoLand.h"
#include "Drv_OpenMV.h"
#include "Can_T265.h"
//#include "Yang_OFMeasure.h"

#define RELATIVE_HEIGHT_CM      (jsdata.valid_of_alt_cm)  //��Ը߶�

extern unsigned char OrdinalNum;
extern char Land_state;
unsigned char LandProgramUnlock_sta = 0;
unsigned char Is_AutoLand = 0;
/*
��������һ������
����˵�����ɻ�һ������
������dT_ms�����ڼ�ʱ
����ֵ����
���ڣ�2022��6��17��
*/
unsigned int LandTimer = 0;
unsigned char Lookflag;

short int t265_x_last = 0;
short int t265_y_last = 0;

void AutoLand(unsigned char dT_ms)
{	
	static unsigned char AutoLand_Num;
	Lookflag = AutoLand_Num;
	if (flag.unlock_sta == 0) // flag.unlock_staΪ�棬���������ɻ����������򡭡���
	{

	}
	else
	{
		if (LandProgramUnlock_sta == 1 && Is_AutoLand == 1) // ��ͨ���п��Ʒɻ�һ������
		{			
			OrdinalNum = 200;
						
			if (RELATIVE_HEIGHT_CM > 17)
			{

				switch(AutoLand_Num)
				{
					case 0:
					{						
						pc_user.pal_dps_set = 0; 		// ȡ����ת
						pc_user.vel_cmps_set_h[0] = 0;  // ȡ��ƽ��
						pc_user.vel_cmps_set_h[1] = 0;
						if(Can_T265.flag == 1){
							t265_x_last = Can_T265.x_Coordinate;
							t265_y_last = Can_T265.y_Coordinate;
						}
						
						
						AutoLand_Num = 1;
					}break;
					case 1:
					{					
						if(Can_T265.flag == 1){
							if(Land_state == 1){
								T265_xy_axis(t265_x_last,t265_y_last);
							}
							else if(Land_state == -1){
								T265_xy_axis2(t265_x_last,t265_y_last);
							}
							
						}
						
						Program_Ctrl_User_Set_Zcmps(-25);						
					}break;
				}
			}
			else
			{
				LandTimer += dT_ms;

				if (LandTimer >= 1000)
				{
					LandTimer = 0;
					flag.unlock_cmd = 0;
				}
			}
					
		}		
	}
}
