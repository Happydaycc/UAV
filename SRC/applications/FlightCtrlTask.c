#include "FlightCtrlTask.h"
#include "Ano_FlightCtrl.h"
#include "Ano_RC.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Ano_Imu.h"
#include "Ano_OF.h"
#include "Drv_icm20602.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_OpenMV.h"
#include "Can_DotStay.h"
#include "AutoTakeOff.h"
#include "AutoLand.h"
#include "LineFollow.h"
#include "Can_DataTransmission.h"
#include "Can_T265.h"
#include "Yang_GYRStraight.h"
#include "Can_Openmv.h"

#define CAR 1
#define Circle 2
#define TAW_CTRL_NUM 98

extern _ano_of_st ano_of;
extern unsigned char KHFlag;
extern unsigned char LandProgramUnlock_sta;
extern unsigned char Is_AutoLand;
extern short int LastDot_X;
extern short int LastDot_Y;
extern unsigned char TaskChoose;
extern float GYRPID_CalculationTargetAngle;

unsigned char SFlag = 1;
unsigned char OrdinalNum = 0;//����ִ�н׶Σ���ɡ����񡢽���
unsigned char DT_data[20] = {0};		//������������F1
unsigned char DT_32data[20] = {0};		//32��������
unsigned char DT_dataF2[30] = {0};		//������������F2
unsigned char K210_data[10] = {0};		//k210��������
unsigned char Ground_data[20] = {0};	//����վ��������

typedef struct
{
	u8 num;
	u8 position1;
	u8 position2;
}num_data_st;
//==��������
num_data_st payload_data[30];	//�洢ɨ�����ݣ�����debug
unsigned short num_i;

u8 MotorFlag = 0;

short int t265_x_target = 0;
short int t265_y_target = 0;
	
unsigned char t265_goto_flag = 0;		//t265�ߵ��־���Ƿ񵽴�Ŀ���
unsigned char servo_flag = 0;		//�����־��1:��  2:��

unsigned char qr_ascill_letter = 0;		//��ά������λ�ñ�ţ���Ϊ��λascill�룬��ʾһ��λ�ñ��
unsigned char qr_ascill_num = 0;

short int t265_y_First = 100;	//�ɻ���ǰ�ɵĹ���������/��ɨ�룬��Ϊ������������������������
short int t265_y_Second = 150;
short int y_qr_delta = 20;		//���˻�ɨ������У����ƶ��ʱy���������������

unsigned int acount = 0;
unsigned int Secondtask_count = 0;

unsigned char FLTask =0;		//����ִ�н׶�
unsigned char fcount = 0;		
unsigned int TaskConfirmCount = 0;//���״̬����
unsigned int TaskTimer_ms = 0;//�������ʱ�䣬�˴�Ϊ����
float Dheight = 0.f;//Ŀ��߶���ʵ�ʸ߶ȵĲ�ֵ
float OFF_CM_my = 125.f;		//���߸߶ȣ������޸�


float my_OFF_CM_up = 125.f;
float my_OFF_CM_down = 86.f;

char Land_state = 1;//��ͷ��t265������Ĺ�ϵ(������׼����)��1��ʾ����-1��ʾ����
char task_state = 0;
char T265_state = 0;
char send_state = 0;

void FlightCtrlTask(unsigned char dT_ms)
{
	DT_data[4] = (short)(TaskTimer_ms);			//1
	DT_data[5] = (short)(TaskTimer_ms) >> 8;
	DT_data[6] = (short)(OrdinalNum);				//2
	DT_data[7] = (short)(OrdinalNum) >> 8;
	DT_data[8] = (short)(jsdata.valid_of_alt_cm);				//3
	DT_data[9] = (short)(jsdata.valid_of_alt_cm) >> 8;	
	DT_data[10] = (short)(pc_user.vel_cmps_set_z);   			//4
	DT_data[11] = (short)(pc_user.vel_cmps_set_z) >> 8;
	DT_data[12] = (short)(Can_T265.x_Coordinate);				//5
	DT_data[13] = (short)(Can_T265.x_Coordinate) >> 8;
	DT_data[14] = (short)(Can_T265.y_Coordinate);				//6
	DT_data[15] = (short)(Can_T265.y_Coordinate) >> 8;
	DT_data[16] = (short)(pc_user.vel_cmps_set_h[0]);			//7
	DT_data[17] = (short)(pc_user.vel_cmps_set_h[0]) >> 8;
	DT_data[18] = (short)(pc_user.vel_cmps_set_h[1]);			//8
	DT_data[19] = (short)(pc_user.vel_cmps_set_h[1]) >> 8;
	
	DT_dataF2[4] = (short)(imu_data.yaw);//1
	DT_dataF2[5] = (short)(imu_data.yaw)>> 8;
	DT_dataF2[6] = (short)(Ground_data[5]);
	DT_dataF2[7] = (short)(Ground_data[5])>> 8;
	DT_dataF2[8] = (short)(Ground_data[6]);
	DT_dataF2[9] = (short)(Ground_data[6])>> 8;
	DT_dataF2[10] = (short)(payload_data[num_i].position1);
	DT_dataF2[11] = (short)(payload_data[num_i].position1)>> 8;
	DT_dataF2[12] = (short)(payload_data[num_i].position2);
	DT_dataF2[13] = (short)(payload_data[num_i].position2)>> 8;
	
	
	DT_dataF2[6] = (short)(FLTask);
	DT_dataF2[7] = (short)(t265_goto_flag);
	Can_shortDataTransfer(DT_data, 0xF1);
	Can_shortDataTransfer(DT_dataF2, 0xF2);

	DT_32data[2] = (short)(jsdata.valid_of_alt_cm);				//9
	DT_32data[3] = (short)(jsdata.valid_of_alt_cm) >> 8;
	DT_32data[4] = (short)(Can_K210.flag);		//2
	DT_32data[5] = (short)(Can_K210.flag) >> 8;	
	DT_32data[6] = (short)(Can_K210.x);				//5
	DT_32data[7] = (short)(Can_K210.x) >> 8;
	DT_32data[8] = (short)(Can_K210.y);				//6
	DT_32data[9] = (short)(Can_K210.y) >> 8;

	//Can_shortDataTo32(DT_32data);
	
	//K210_data[0] = 0x31;
	//K210_data[1] = 0x01;
	if (CH_N[7] < 0){                                    	//����һ                  											
		//Can_shortDataToK210(K210_data); //����
		//Can_short_twoDataToK210(0x31,0x01);//������
	}
	//Can_short_twoDataToK210(0x31,0x02);//�������
	//Can_short_twoDataToK210(0x31,0x02);//�������
	
	Ground_data[4] = Can_QRScan.flag;//��ά��flag
	//Ground_data[5] = 0;//λ�ñ��  ascill
	Ground_data[6] = 9;//λ�ñ��
	//Ground_data[7] = Can_QRScan.payload;//��ά������
	Ground_data[13] = 0;
	
	
//	if(Can_QRScan.flag ==1 || Can_QRScan.flag ==2){
//		Can_QRScan.last_payload = Can_QRScan.payload;//��¼ɨ������
//	}
	
	//ģʽ�л�
	if(T265_state == 0){
		if(Can_T265.flag == 1){
			///Can_shortDataToGround(Ground_data,0x03);
			Can_short_twoDataToK210(0x22,0x02);
			T265_state = 1;
		}
	}
//		if (CH_N[6] < -300){
//			
//		}
		
	    //����һ                  											
		if(task_state == 0){
			if (CH_N[6] > 300){
				if(Can_QRScan.flag ==1 || Can_QRScan.flag ==2){
					Can_QRScan.Secondtask_payload = Can_QRScan.payload;//��¼ɨ������
					Ground_data[7] = Can_QRScan.payload;
					Can_shortDataToGround(Ground_data,0x04);//ͬʱ��������վ��ѯ
					Can_short_twoDataToK210(0x33,Can_QRScan.payload);
					Can_QRScan.flag = 0;
					task_state = 1;//��ת��״̬1���ȴ�
				}
			}
		}
		else if(task_state == 1){
			//�����ѯ�������ظ������˻������͵�����վ��ʾ�����ֶ��������
			if(Can_QRScan.Secondtask_flag == 1){
				if(Secondtask_count < 10){
					Ground_data[13] = 0xAA;
					//�жϵ���վ��ѯ���ı���еڶ�λ������1��2��3������4��5��6
					//�����1��2��3��1m3
					if(Can_QRScan.Secondtask_position2 == 1||Can_QRScan.Secondtask_position2 == 2||Can_QRScan.Secondtask_position2 == 3){
						OFF_CM_my = my_OFF_CM_up;
					}
					else if(Can_QRScan.Secondtask_position2 == 4||Can_QRScan.Secondtask_position2 == 5||Can_QRScan.Secondtask_position2 == 6){
						OFF_CM_my = my_OFF_CM_down;
					}
					
					Can_shortDataToGround(Ground_data,0x04);
					Can_short_twoDataToK210(0x0,0x0);
					task_state = 2;
					Secondtask_count++;
				}
			}		
		}
		else if(task_state == 2){
			//�����ѯ�������ظ������˻������͵�����վ��ʾ�����ֶ��������
			if(Can_QRScan.Secondtask_flag == 1){
				Ground_data[13] = 0xAA;
				Can_shortDataToGround(Ground_data,0x04);
				Can_short_twoDataToK210(0x33,Can_QRScan.payload);
				task_state = 3;
			}		
		}
		
	
	
	//Can_shortDataToGround(Ground_data,0x03);//t265��ѯ
	/****************************************��������****************************************************************/
	Dheight = OFF_CM - RELATIVE_HEIGHT_CM;//�߶Ȳ�ֵ
	
	
	
	if(flag.unlock_sta == 1)
	{
		Remote_Control(dT_ms);
		AutoLand(dT_ms);
//		if(KHFlag == 0){
//			Can_shortDataToGround(Ground_data,0x03);
//		}
		if(KHFlag == 1 && LandProgramUnlock_sta == 0)//����
		{
			KeepHeight(OFF_CM_my);
		};		
		switch(OrdinalNum)
		{
			case 0:                                             //�׶�һһ�����
			{
				
				if (SFlag == 1)
				{
					SFlag = 0;				
					TaskConfirmCount = 0;
				}
				AutoTakeOff(dT_ms);                                 
				if(KHFlag == 1)                                    
				{
					if (TaskConfirmCount <= 10)
					{
						
						TaskConfirmCount ++;
					}
					else
					{
						TaskConfirmCount = 0;
						TaskTimer_ms = 0;					
						SFlag = 1;	
						if(TaskChoose == 1){                                    	//����һ                  											
							OrdinalNum = 50;
						}
						else if(TaskChoose == 2){																	//�����		
							OrdinalNum = 60;
						}			
						
					}				
				};
			}break;
/**********************************************************************************************/	
//����һ,ȫ��ɨ��
			case 50: //��ͣ5s����
			{		

				if (SFlag == 1)
				{
					SFlag = 0;
					TaskTimer_ms = 0;
				}		
				switch(FLTask){			
					case 0:{		
						
						FLTask = 1;
					}break;
					case 1:{
						//��ɶ�3��
						if(TaskTimer_ms  <= 2000){
							TaskTimer_ms += dT_ms;
							T265_x_axis_freeze(0, Can_T265.x_Coordinate);
							T265_y_axis_freeze(0, Can_T265.y_Coordinate);
						}
						else{//������ʶ������������ǰ��һ�ξ����ٿ�ʼɨ�룬����ɨ��1��14
							t265_x_target = 0;
							t265_y_target =  70;
							T265_xy_axis(t265_x_target,t265_y_target);
							if(Can_T265.y_Coordinate > 50){
								TaskTimer_ms = 0;
								FLTask = 11;
							}
						}
					}break;
					
					
					case 11:{
						send_state = 1;
						if(MotorFlag == 0){//k210�������
							
							Can_short_twoDataToK210(0x11,0x22);
							MotorFlag++;
						}
						//ǰ����2m
						t265_x_target =  0; 	//���˻���a��ľ���̶�
						t265_y_target =  210;	//
						
						Ground_data[5] = 'A';//λ�ñ��  ascill  A
//						if(Can_T265.y_Coordinate  < 30){
//							Can_QRScan.flag = 0;
//						}
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						if(Can_T265.y_Coordinate  < t265_y_First){
							qr_ascill_num = 0x03;//���������ϣ����ΪA3	
						}else if(Can_T265.y_Coordinate < t265_y_Second){
							qr_ascill_num = 0x02;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x01;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;

						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң����򱱷�ͬʱɨ�룬��y����� 200cm
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 12;
						}
						
					}break;
					
					case 12:{
						//�߶Ƚ���  ׼��ɨ��ڶ���
						OFF_CM_my = my_OFF_CM_down;	//�߶�����40cm
						T265_xy_axis(t265_x_target,t265_y_target);	//�߶��½�����������ˮƽ�ȶ�
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 13;
						}
						
					}break;
					
					//ɨ��a��ɨc��
					case 13:{
						//ɨ��4��5��6
						t265_y_target = -20;
						if(Can_T265.y_Coordinate < 30){
							Ground_data[5] = 'C';
						}
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x04;//���������ϣ����ΪA3
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x05;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x06;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ����
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 14;
						}
					}break;
					
					case 14:{
						t265_x_target =  180;	//ǰ��c��
						Ground_data[5] = 'C';
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ����
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 2;
						}
					}break;
					
					
					case 2:{
						//ɨc��4��5��6
							//t265_x_target =  0; 	//���˻���a��ľ���̶�
							t265_y_target =  210;	//
							
							Ground_data[5] = 'C';//λ�ñ��  ascill  A
							if(Can_T265.y_Coordinate  < t265_y_First){
								qr_ascill_num = 0x06;//���������ϣ����ΪA3	
							}else if(Can_T265.y_Coordinate < t265_y_Second){
								qr_ascill_num = 0x05;//���������ϣ����ΪA3
							}
							else{
								qr_ascill_num = 0x04;//���������ϣ����ΪA3
							}
							Ground_data[6] = qr_ascill_num;

							t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң����򱱷�ͬʱɨ�룬��y����� 200cm
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								pc_user.vel_cmps_set_h[0] = 0;
								pc_user.vel_cmps_set_h[1] = 0;
								TaskTimer_ms = 0;
								FLTask = 21;
							}
					}break;
					case 21:{
						//�߶�  ׼��ɨ��c�ĵڶ���
						OFF_CM_my = my_OFF_CM_up;	//�߶�����
						T265_xy_axis(t265_x_target,t265_y_target);	//�߶��½�����������ˮƽ�ȶ�
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 22;
						}
						
					}break;
					case 22:{
						//ɨc��1��2��3
						t265_y_target =  50;	//ɨc��
						
						Ground_data[5] = 'C';//λ�ñ��  ascill  c
						
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x01;//���������ϣ����ΪA3
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x02;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x03;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;
							
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ����
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 23;
						}
					}break;
					case 23:{//��ת180�ȣ�ɨ�ڶ���
						
						//ȡ���˴����ƣ��ɻ���ͷ�򶫣�ɨ��a��������ɨb�棬�Ƚ���ͷ��ת180��
						//��ǰ��Ϊ0�ȣ�˳ʱ����תΪ�������180  ��ʱ�뷽��Ϊ������С-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//��ת
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 3;
							SFlag = 1;
						}
						
					}break;
					
					
					case 3:{//B��
						//ɨb��1��2��3
						//��ת֮��Ϊ����
						t265_x_target = 200;
						if(TaskTimer_ms  <= 1000){//��ת�󶨵�
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
							t265_y_target = 210;
							
							Ground_data[5] = 'B';//λ�ñ��  ascill  B
							
							//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
							if(Can_T265.y_Coordinate  < t265_y_First){
								qr_ascill_num = 0x01;//���������ϣ����ΪA3	
							}else if(Can_T265.y_Coordinate < t265_y_Second){
								qr_ascill_num = 0x02;//���������ϣ����ΪA3
							}
							else{
								qr_ascill_num = 0x03;//���������ϣ����ΪA3
							}
							Ground_data[6] = qr_ascill_num;
							//Can_shortDataToGround(Ground_data,0x05);//���ڷ���������վ���洢
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);		//�򱱷ɣ�ɨ��b��  
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								TaskTimer_ms = 0;
								SFlag = 1;
								FLTask = 31;
							}
						}
					}break;
					case 31:{
						//�߶Ƚ���  
						OFF_CM_my = my_OFF_CM_down;	//�߶�
						T265_xy_axis2(t265_x_target,t265_y_target);	//�߶��½�����������ˮƽ�ȶ�
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 32;
						}
						
					}break;
					case 32:{
						//ɨ��b��4��5��6
						t265_y_target =  -20;
						
						if(Can_T265.y_Coordinate  > t265_y_Second ){
							qr_ascill_num = 0x06;//���������ϣ����ΪA3	
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x05;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x04;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 33;
						}
						
					}break;
					case 33:{
						//ǰ��d��
						t265_x_target = 400;	
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 4;
						}
						
					}break;
					case 4:{
						//ɨ��d��4��5��6
						t265_y_target = 210;	
						Ground_data[5] = 'D';//λ�ñ��  ascill  B
						if(Can_T265.y_Coordinate  < t265_y_First){
							qr_ascill_num = 0x04;//���������ϣ����ΪA3	
						}else if(Can_T265.y_Coordinate < t265_y_Second){
							qr_ascill_num = 0x05;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x06;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 41;
						}
						
					}break;
					case 41:{
						//�߶Ƚ���  ׼��ɨ��d�ĵ�һ��
						OFF_CM_my = my_OFF_CM_up;	//�߶�����40cm
						T265_xy_axis2(t265_x_target,t265_y_target);	//�߶��½�����������ˮƽ�ȶ�
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 42;
						}
						
					}break;
					case 42:{
						//d��1��2��3
						t265_y_target = 50;
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x03;//���������ϣ����ΪA3	
						}else if(Can_T265.y_Coordinate > t265_y_First ){
							qr_ascill_num = 0x02;//���������ϣ����ΪA3
						}
						else{
							qr_ascill_num = 0x01;//���������ϣ����ΪA3
						}
						Ground_data[6] = qr_ascill_num;
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 6;
						}
						
					}break;
					case 6:{
						//ǰ�������
						t265_x_target = 360;
						t265_y_target = 250;	//�˻���
						
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ����
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 7;
						}
					}break;
					case 7:{
						Land_state = -1;//��ʾ���򣬽���ʱpidʹ�÷�����
						TaskTimer_ms = 0;
						OrdinalNum = 99;
					}break;
				}
				
				if(send_state == 1){
					if(Can_QRScan.flag == 1 || Can_QRScan.flag == 2){
						Ground_data[7] = Can_QRScan.payload;
						Can_shortDataToGround(Ground_data,0x05);//���ڷ���������վ���洢
						Can_QRScan.flag = 0;
					}
				}
				else{
					Can_QRScan.flag = 0;
				}
			}break;
/**************************************************************************************/
//�������ǰ�ɶ�С��
			case 60: //��ͣ5s����
			{			
				if (SFlag == 1)
					{
						SFlag = 0;
						TaskTimer_ms = 0;
					}	
				switch(FLTask){
					case 0:{
						//��ɶ���
						if(TaskTimer_ms  <= 2000){
							TaskTimer_ms += dT_ms;
							T265_x_axis_freeze(0, Can_T265.x_Coordinate);
							T265_y_axis_freeze(0, Can_T265.y_Coordinate);
						}else{//�жϵ���վ��ѯ���ı����ABCD�е���һ��
							if(Can_QRScan.Secondtask_position1 == 'A'){
								FLTask = 1;
							}
							else if(Can_QRScan.Secondtask_position1 == 'B'){
								FLTask = 2;
							}
							else if(Can_QRScan.Secondtask_position1 == 'C'){
								FLTask = 3;
							}
							else if(Can_QRScan.Secondtask_position1 == 'D'){
								FLTask = 4;
							}
							else{
								FLTask = 1;
							}
							
							
							TaskTimer_ms = 0;
						}					
					}break;
					case 1:{				
						t265_x_target = 0;	//a��
						t265_y_target = 0;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 11;
						}
					}break;
					case 11:{				
						t265_y_target = 280;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 12;
						}
					}break;
					case 12:{				
						t265_x_target = 340;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 6;
						}
					}break;
					
					case 2:{				
						t265_x_target = 0;	//b��
						t265_y_target = -20;
						
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							
							FLTask = 21;
						}
					}break;
					
					case 21:{				
						t265_x_target = 165;	//ǰ��b��
						//t265_y_target = -20;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 22;
						}
					}break;
					case 22:{				
						//t265_x_target = 165;	//ǰ��b��
						t265_y_target = 50;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							FLTask = 23;
						}
					}break;
					case 23:{				
						//ȡ���˴����ƣ��ɻ���ͷ�򶫣�ɨ��a��������ɨb�棬�Ƚ���ͷ��ת180��
						//��ǰ��Ϊ0�ȣ�˳ʱ����תΪ�������180  ��ʱ�뷽��Ϊ������С-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//��ת
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 24;
							SFlag = 1;
						}
					}break;
					case 24:{		
						t265_x_target = 200;
						if(TaskTimer_ms  <= 1000){//��ת�󶨵�
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
							//t265_x_target = 165;	//ɨ��b��
							t265_y_target = 280;
							//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								TaskTimer_ms = 0;
								Land_state = -1;
								FLTask = 25;
							}
						}
					}break;
					case 25:{
						t265_x_target = 360;	//ǰ�������
						//t265_y_target = 280;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = -1;
							FLTask = 26;
						}
					}break;
					case 26:{		
						
						//t265_x_target = 360;	//
						t265_y_target = 250;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = -1;
							FLTask = 7;
						}
						
					}break;
					
					case 3:{				
						t265_x_target = 0;	//c��
						t265_y_target = 0;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 31;
						}
					}break;
					case 31:{				
						t265_y_target = -20;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 32;
						}
					}break;
					case 32:{				
						t265_x_target = 170;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 33;
						}
					}break;
					case 33:{				
						t265_y_target = 280;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 34;
						}
					}break;
					case 34:{				
						t265_x_target = 340;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 6;
						}
					}break;
					
					case 4:{				
						t265_x_target = 0;	//d��
						t265_y_target = 0;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 41;
						}
					}break;
					case 41:{				
						//t265_x_target = 0;	//d��
						t265_y_target = -20;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 42;
						}
					}break;
					case 42:{				
						t265_x_target = 380;	
						//t265_y_target = -20;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 43;
						}
					}break;
					case 43:{				
						//t265_x_target = 380;	
						t265_y_target = 50;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 44;
						}
					}break;
					case 44:{				
						//ȡ���˴����ƣ��ɻ���ͷ�򶫣�ɨ��a��������ɨb�棬�Ƚ���ͷ��ת180��
						//��ǰ��Ϊ0�ȣ�˳ʱ����תΪ�������180  ��ʱ�뷽��Ϊ������С-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//��ת
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 45;
							SFlag = 1;
						}
					}break;
					case 45:{		
						t265_x_target = 400;
						if(TaskTimer_ms  <= 1000){//��ת�󶨵�
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
//							t265_y_target = 280;
							//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
//							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
//							if(t265_goto_flag == 1){
//								t265_goto_flag = 0;
//								TaskTimer_ms = 0;
//								Land_state = -1;
//								FLTask = 46;
//							}
							Land_state = -1;
							FLTask = 46;
						}
					}break;
					case 46:{				
						//t265_x_target = 380;
						t265_y_target = 250;
						//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 6;
						}
					}break;
					
					case 6:{
						//ͳһǰ��ǰ��
						
						
						t265_y_target = 250;
						if(Land_state == 1){//û��ת
							t265_x_target = 340;
							//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
							t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						}
						else if(Land_state == -1){//��ת�����ᷴ��
							t265_x_target = 360;
							//����ɨûɨ�������д���ͨ��/ֻ��ɨ�����ʱ����  �����ݷ��͸�����վ
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//��ͷ���ң������Ϸ�ͬʱɨ��
						}
						if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								TaskTimer_ms = 0;
								FLTask = 7;
						}	
					}break;
					case 7:{
						TaskTimer_ms = 0;
						OrdinalNum = 99;	
					}break;
				}
				if(Can_QRScan.flag == 1 || Can_QRScan.flag == 2){
						Ground_data[5] = Can_QRScan.Secondtask_position1;
						Ground_data[6] = Can_QRScan.Secondtask_position2;
						Ground_data[7] = Can_QRScan.payload;
						Can_shortDataToGround(Ground_data,0x04);//���ڷ���������վ���洢
						Can_QRScan.flag = 0;
					}
			}break;		
/*********************************************************************************************/
			
			case 99:
			{
				LandProgramUnlock_sta = 1;
				Is_AutoLand = Yes;
			}break;
		};
	};
};

