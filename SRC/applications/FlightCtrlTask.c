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
unsigned char OrdinalNum = 0;//任务执行阶段，起飞、任务、降落
unsigned char DT_data[20] = {0};		//匿名数传发送F1
unsigned char DT_32data[20] = {0};		//32数传发送
unsigned char DT_dataF2[30] = {0};		//匿名数传发送F2
unsigned char K210_data[10] = {0};		//k210数传发送
unsigned char Ground_data[20] = {0};	//地面站数传发送

typedef struct
{
	u8 num;
	u8 position1;
	u8 position2;
}num_data_st;
//==数据声明
num_data_st payload_data[30];	//存储扫码数据，方便debug
unsigned short num_i;

u8 MotorFlag = 0;

short int t265_x_target = 0;
short int t265_y_target = 0;
	
unsigned char t265_goto_flag = 0;		//t265走点标志，是否到达目标点
unsigned char servo_flag = 0;		//舵机标志，1:上  2:下

unsigned char qr_ascill_letter = 0;		//二维码所在位置编号，分为两位ascill码，表示一个位置编号
unsigned char qr_ascill_num = 0;

short int t265_y_First = 100;	//飞机向前飞的过程中向左/右扫码，分为三个区域，用两条坐标线区分
short int t265_y_Second = 150;
short int y_qr_delta = 20;		//无人机扫码过程中，控制舵机时y轴相对于上面两个

unsigned int acount = 0;
unsigned int Secondtask_count = 0;

unsigned char FLTask =0;		//任务执行阶段
unsigned char fcount = 0;		
unsigned int TaskConfirmCount = 0;//起飞状态计数
unsigned int TaskTimer_ms = 0;//任务持续时间，此处为定高
float Dheight = 0.f;//目标高度与实际高度的差值
float OFF_CM_my = 125.f;		//定高高度，谨慎修改


float my_OFF_CM_up = 125.f;
float my_OFF_CM_down = 86.f;

char Land_state = 1;//机头与t265坐标轴的关系(辅助精准降落)，1表示正向，-1表示反向
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
	if (CH_N[7] < 0){                                    	//任务一                  											
		//Can_shortDataToK210(K210_data); //激光
		//Can_short_twoDataToK210(0x31,0x01);//蜂鸣器
	}
	//Can_short_twoDataToK210(0x31,0x02);//舵机向上
	//Can_short_twoDataToK210(0x31,0x02);//舵机向上
	
	Ground_data[4] = Can_QRScan.flag;//二维码flag
	//Ground_data[5] = 0;//位置编号  ascill
	Ground_data[6] = 9;//位置编号
	//Ground_data[7] = Can_QRScan.payload;//二维码数字
	Ground_data[13] = 0;
	
	
//	if(Can_QRScan.flag ==1 || Can_QRScan.flag ==2){
//		Can_QRScan.last_payload = Can_QRScan.payload;//记录扫到的数
//	}
	
	//模式切换
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
		
	    //任务一                  											
		if(task_state == 0){
			if (CH_N[6] > 300){
				if(Can_QRScan.flag ==1 || Can_QRScan.flag ==2){
					Can_QRScan.Secondtask_payload = Can_QRScan.payload;//记录扫到的数
					Ground_data[7] = Can_QRScan.payload;
					Can_shortDataToGround(Ground_data,0x04);//同时发给地面站查询
					Can_short_twoDataToK210(0x33,Can_QRScan.payload);
					Can_QRScan.flag = 0;
					task_state = 1;//跳转到状态1，等待
				}
			}
		}
		else if(task_state == 1){
			//如果查询到并返回给了无人机，发送到地面站显示，并手动解锁起飞
			if(Can_QRScan.Secondtask_flag == 1){
				if(Secondtask_count < 10){
					Ground_data[13] = 0xAA;
					//判断地面站查询到的编号中第二位数据是1、2、3，还是4、5、6
					//如果是1、2、3飞1m3
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
			//如果查询到并返回给了无人机，发送到地面站显示，并手动解锁起飞
			if(Can_QRScan.Secondtask_flag == 1){
				Ground_data[13] = 0xAA;
				Can_shortDataToGround(Ground_data,0x04);
				Can_short_twoDataToK210(0x33,Can_QRScan.payload);
				task_state = 3;
			}		
		}
		
	
	
	//Can_shortDataToGround(Ground_data,0x03);//t265查询
	/****************************************数传部分****************************************************************/
	Dheight = OFF_CM - RELATIVE_HEIGHT_CM;//高度差值
	
	
	
	if(flag.unlock_sta == 1)
	{
		Remote_Control(dT_ms);
		AutoLand(dT_ms);
//		if(KHFlag == 0){
//			Can_shortDataToGround(Ground_data,0x03);
//		}
		if(KHFlag == 1 && LandProgramUnlock_sta == 0)//定高
		{
			KeepHeight(OFF_CM_my);
		};		
		switch(OrdinalNum)
		{
			case 0:                                             //阶段一一键起飞
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
						if(TaskChoose == 1){                                    	//任务一                  											
							OrdinalNum = 50;
						}
						else if(TaskChoose == 2){																	//任务二		
							OrdinalNum = 60;
						}			
						
					}				
				};
			}break;
/**********************************************************************************************/	
//任务一,全局扫描
			case 50: //悬停5s后降落
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
						//起飞定3秒
						if(TaskTimer_ms  <= 2000){
							TaskTimer_ms += dT_ms;
							T265_x_axis_freeze(0, Can_T265.x_Coordinate);
							T265_y_axis_freeze(0, Can_T265.y_Coordinate);
						}
						else{//处理误识别的情况，先向前走一段距离再开始扫码，避免扫地1和14
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
						if(MotorFlag == 0){//k210清空数组
							
							Can_short_twoDataToK210(0x11,0x22);
							MotorFlag++;
						}
						//前进到2m
						t265_x_target =  0; 	//无人机到a面的距离固定
						t265_y_target =  210;	//
						
						Ground_data[5] = 'A';//位置编号  ascill  A
//						if(Can_T265.y_Coordinate  < 30){
//							Can_QRScan.flag = 0;
//						}
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						if(Can_T265.y_Coordinate  < t265_y_First){
							qr_ascill_num = 0x03;//如果舵机在上，编号为A3	
						}else if(Can_T265.y_Coordinate < t265_y_Second){
							qr_ascill_num = 0x02;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x01;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;

						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向北飞同时扫码，即y轴左侧 200cm
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 12;
						}
						
					}break;
					
					case 12:{
						//高度降低  准备扫描第二行
						OFF_CM_my = my_OFF_CM_down;	//高度上升40cm
						T265_xy_axis(t265_x_target,t265_y_target);	//高度下降过程中让其水平稳定
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 13;
						}
						
					}break;
					
					//扫完a面扫c面
					case 13:{
						//扫面4、5、6
						t265_y_target = -20;
						if(Can_T265.y_Coordinate < 30){
							Ground_data[5] = 'C';
						}
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x04;//如果舵机在上，编号为A3
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x05;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x06;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 14;
						}
					}break;
					
					case 14:{
						t265_x_target =  180;	//前往c面
						Ground_data[5] = 'C';
						//无论扫没扫到都进行串口通信/只在扫到的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 2;
						}
					}break;
					
					
					case 2:{
						//扫c面4、5、6
							//t265_x_target =  0; 	//无人机到a面的距离固定
							t265_y_target =  210;	//
							
							Ground_data[5] = 'C';//位置编号  ascill  A
							if(Can_T265.y_Coordinate  < t265_y_First){
								qr_ascill_num = 0x06;//如果舵机在上，编号为A3	
							}else if(Can_T265.y_Coordinate < t265_y_Second){
								qr_ascill_num = 0x05;//如果舵机在上，编号为A3
							}
							else{
								qr_ascill_num = 0x04;//如果舵机在上，编号为A3
							}
							Ground_data[6] = qr_ascill_num;

							t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向北飞同时扫码，即y轴左侧 200cm
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								pc_user.vel_cmps_set_h[0] = 0;
								pc_user.vel_cmps_set_h[1] = 0;
								TaskTimer_ms = 0;
								FLTask = 21;
							}
					}break;
					case 21:{
						//高度  准备扫描c的第二行
						OFF_CM_my = my_OFF_CM_up;	//高度上升
						T265_xy_axis(t265_x_target,t265_y_target);	//高度下降过程中让其水平稳定
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 22;
						}
						
					}break;
					case 22:{
						//扫c面1、2、3
						t265_y_target =  50;	//扫c面
						
						Ground_data[5] = 'C';//位置编号  ascill  c
						
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x01;//如果舵机在上，编号为A3
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x02;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x03;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;
							
						//无论扫没扫到都进行串口通信/只在扫到的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 23;
						}
					}break;
					case 23:{//旋转180度，扫第二面
						
						//取消了磁力计，飞机机头向东，扫完a面后接下来扫b面，先将机头旋转180度
						//正前方为0度，顺时针旋转为正，最大180  逆时针方向为负，最小-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//旋转
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 3;
							SFlag = 1;
						}
						
					}break;
					
					
					case 3:{//B面
						//扫b面1、2、3
						//旋转之后为反向
						t265_x_target = 200;
						if(TaskTimer_ms  <= 1000){//旋转后定点
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
							t265_y_target = 210;
							
							Ground_data[5] = 'B';//位置编号  ascill  B
							
							//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
							if(Can_T265.y_Coordinate  < t265_y_First){
								qr_ascill_num = 0x01;//如果舵机在上，编号为A3	
							}else if(Can_T265.y_Coordinate < t265_y_Second){
								qr_ascill_num = 0x02;//如果舵机在上，编号为A3
							}
							else{
								qr_ascill_num = 0x03;//如果舵机在上，编号为A3
							}
							Ground_data[6] = qr_ascill_num;
							//Can_shortDataToGround(Ground_data,0x05);//串口发送至地面站，存储
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);		//向北飞，扫描b面  
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								TaskTimer_ms = 0;
								SFlag = 1;
								FLTask = 31;
							}
						}
					}break;
					case 31:{
						//高度降低  
						OFF_CM_my = my_OFF_CM_down;	//高度
						T265_xy_axis2(t265_x_target,t265_y_target);	//高度下降过程中让其水平稳定
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 32;
						}
						
					}break;
					case 32:{
						//扫描b面4、5、6
						t265_y_target =  -20;
						
						if(Can_T265.y_Coordinate  > t265_y_Second ){
							qr_ascill_num = 0x06;//如果舵机在上，编号为A3	
						}else if(Can_T265.y_Coordinate > t265_y_First){
							qr_ascill_num = 0x05;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x04;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 33;
						}
						
					}break;
					case 33:{
						//前往d面
						t265_x_target = 400;	
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 4;
						}
						
					}break;
					case 4:{
						//扫描d面4、5、6
						t265_y_target = 210;	
						Ground_data[5] = 'D';//位置编号  ascill  B
						if(Can_T265.y_Coordinate  < t265_y_First){
							qr_ascill_num = 0x04;//如果舵机在上，编号为A3	
						}else if(Can_T265.y_Coordinate < t265_y_Second){
							qr_ascill_num = 0x05;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x06;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 41;
						}
						
					}break;
					case 41:{
						//高度降低  准备扫描d的第一行
						OFF_CM_my = my_OFF_CM_up;	//高度上升40cm
						T265_xy_axis2(t265_x_target,t265_y_target);	//高度下降过程中让其水平稳定
						if( ABS(OFF_CM_my - RELATIVE_HEIGHT_CM) <= 8){
							FLTask = 42;
						}
						
					}break;
					case 42:{
						//d面1、2、3
						t265_y_target = 50;
						if(Can_T265.y_Coordinate  > t265_y_Second){
							qr_ascill_num = 0x03;//如果舵机在上，编号为A3	
						}else if(Can_T265.y_Coordinate > t265_y_First ){
							qr_ascill_num = 0x02;//如果舵机在上，编号为A3
						}
						else{
							qr_ascill_num = 0x01;//如果舵机在上，编号为A3
						}
						Ground_data[6] = qr_ascill_num;
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 6;
						}
						
					}break;
					case 6:{
						//前往降落点
						t265_x_target = 360;
						t265_y_target = 250;	//退回来
						
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 7;
						}
					}break;
					case 7:{
						Land_state = -1;//表示反向，降落时pid使用反向函数
						TaskTimer_ms = 0;
						OrdinalNum = 99;
					}break;
				}
				
				if(send_state == 1){
					if(Can_QRScan.flag == 1 || Can_QRScan.flag == 2){
						Ground_data[7] = Can_QRScan.payload;
						Can_shortDataToGround(Ground_data,0x05);//串口发送至地面站，存储
						Can_QRScan.flag = 0;
					}
				}
				else{
					Can_QRScan.flag = 0;
				}
			}break;
/**************************************************************************************/
//任务二往前飞定小车
			case 60: //悬停5s后降落
			{			
				if (SFlag == 1)
					{
						SFlag = 0;
						TaskTimer_ms = 0;
					}	
				switch(FLTask){
					case 0:{
						//起飞定点
						if(TaskTimer_ms  <= 2000){
							TaskTimer_ms += dT_ms;
							T265_x_axis_freeze(0, Can_T265.x_Coordinate);
							T265_y_axis_freeze(0, Can_T265.y_Coordinate);
						}else{//判断地面站查询到的编号是ABCD中的哪一个
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
						t265_x_target = 0;	//a面
						t265_y_target = 0;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 11;
						}
					}break;
					case 11:{				
						t265_y_target = 280;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 12;
						}
					}break;
					case 12:{				
						t265_x_target = 340;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 6;
						}
					}break;
					
					case 2:{				
						t265_x_target = 0;	//b面
						t265_y_target = -20;
						
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							
							FLTask = 21;
						}
					}break;
					
					case 21:{				
						t265_x_target = 165;	//前往b面
						//t265_y_target = -20;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 22;
						}
					}break;
					case 22:{				
						//t265_x_target = 165;	//前往b面
						t265_y_target = 50;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							FLTask = 23;
						}
					}break;
					case 23:{				
						//取消了磁力计，飞机机头向东，扫完a面后接下来扫b面，先将机头旋转180度
						//正前方为0度，顺时针旋转为正，最大180  逆时针方向为负，最小-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//旋转
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 24;
							SFlag = 1;
						}
					}break;
					case 24:{		
						t265_x_target = 200;
						if(TaskTimer_ms  <= 1000){//旋转后定点
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
							//t265_x_target = 165;	//扫描b面
							t265_y_target = 280;
							//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
							if(t265_goto_flag == 1){
								t265_goto_flag = 0;
								TaskTimer_ms = 0;
								Land_state = -1;
								FLTask = 25;
							}
						}
					}break;
					case 25:{
						t265_x_target = 360;	//前往降落点
						//t265_y_target = 280;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
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
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = -1;
							FLTask = 7;
						}
						
					}break;
					
					case 3:{				
						t265_x_target = 0;	//c面
						t265_y_target = 0;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 31;
						}
					}break;
					case 31:{				
						t265_y_target = -20;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 32;
						}
					}break;
					case 32:{				
						t265_x_target = 170;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 33;
						}
					}break;
					case 33:{				
						t265_y_target = 280;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 34;
						}
					}break;
					case 34:{				
						t265_x_target = 340;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							Land_state = 1;
							FLTask = 6;
						}
					}break;
					
					case 4:{				
						t265_x_target = 0;	//d面
						t265_y_target = 0;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 41;
						}
					}break;
					case 41:{				
						//t265_x_target = 0;	//d面
						t265_y_target = -20;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 42;
						}
					}break;
					case 42:{				
						t265_x_target = 380;	
						//t265_y_target = -20;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 43;
						}
					}break;
					case 43:{				
						//t265_x_target = 380;	
						t265_y_target = 50;
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							pc_user.vel_cmps_set_h[0] = 0;
							pc_user.vel_cmps_set_h[1] = 0;
							TaskTimer_ms = 0;
							FLTask = 44;
						}
					}break;
					case 44:{				
						//取消了磁力计，飞机机头向东，扫完a面后接下来扫b面，先将机头旋转180度
						//正前方为0度，顺时针旋转为正，最大180  逆时针方向为负，最小-179
						GYRPID_CalculationTargetAngle = 180;
						GYR_Straight();			//旋转
						if(ABS(GYRPID_CalculationTargetAngle - imu_data.yaw) <= 8){
							pc_user.pal_dps_set = 0;
							TaskTimer_ms = 0;
							FLTask = 45;
							SFlag = 1;
						}
					}break;
					case 45:{		
						t265_x_target = 400;
						if(TaskTimer_ms  <= 1000){//旋转后定点
							T265_xy_axis2(t265_x_target,t265_y_target);
							TaskTimer_ms += dT_ms;
						}else{
//							t265_y_target = 280;
							//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
//							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
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
						//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
						t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						if(t265_goto_flag == 1){
							t265_goto_flag = 0;
							TaskTimer_ms = 0;
							FLTask = 6;
						}
					}break;
					
					case 6:{
						//统一前往前线
						
						
						t265_y_target = 250;
						if(Land_state == 1){//没旋转
							t265_x_target = 340;
							//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
							t265_goto_flag = T265_xy_goto(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
						}
						else if(Land_state == -1){//旋转过，轴反向
							t265_x_target = 360;
							//无论扫没扫到都进行串口通信/只在扫到码的时候发送  将数据发送给地面站
							t265_goto_flag = T265_xy_goto2(t265_x_target,t265_y_target);	//机头朝右，先向南飞同时扫码
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
						Can_shortDataToGround(Ground_data,0x04);//串口发送至地面站，存储
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

