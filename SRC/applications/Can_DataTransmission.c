

#include "Drv_usart.h"
#include "Drv_OpenMV.h"
#include "FlightCtrlTask.h"

unsigned char DataReceive[5] = {0};
unsigned char Usart_Data4Car[3] = {0};
unsigned char DT_K210data[20] = {0};


//extern unsigned short num_i;
extern unsigned char servo_flag;

void Can_DataAnalysis(unsigned char Byte)
{	
	static unsigned char i;
	unsigned char sum_data = 0;
	if (i == 0)
	{
		if (Byte == 0xAA)
		{
			DataReceive[i] = Byte;
			i ++;
		}
	}
	else if (i == 1)
	{
		if (Byte == 0xFF)
		{
			DataReceive[i] = Byte;
			i ++;
		}
	}
	else if (i >= 2 && i <= 3)
	{
		DataReceive[i] = Byte;
		i ++;
	}
	else if (i == 4)
	{
		for (i = 0; i < 4; i ++)
		{
			sum_data += DataReceive[i];
		}
		i = 4;
		
		if (sum_data == Byte)
		{
			DataReceive[i] = Byte;
			
			if (DataReceive[2] == 0xF1)		// 起飞标志位
			{
				Usart_Data4Car[0] = DataReceive[3];
			}
			else if (DataReceive[2] == 0xF2)	// 火源标志位
			{
				Usart_Data4Car[1] = DataReceive[3];
			}
			else if (DataReceive[2] == 0xF3)	// 一键起飞降落位
			{
				Usart_Data4Car[2] = DataReceive[3];
			}
		}
		
		i = 0;
	}
}


//数传发送数据打包
// int16_t数据这么存，short i; Data[4] = i; Data[5] = i >> 8;
// 若上位机是匿名上位机，对于Data[4]、Data[5]来说，对应ID的数据位1，选int16
// Data[4]~Data[13]是数据位

void Can_shortDataTransfer(unsigned char *Data, unsigned char ID)
{
	unsigned char i = 0;
	unsigned char sum_check = 0;
	unsigned char add_check = 0;
	
	Data[0] = 0xAA;
	Data[1] = 0xFF;
	Data[2] = ID;			
	Data[3] = 16; 			// 1 char = 1 Byte, 1 short = 2 Byte 
	for (i = 0; i < 20; i ++)
	{
		sum_check += Data[i];
		add_check += sum_check;
	}	
	Data[20] = sum_check;
	Data[21] = add_check;
	
	//UT5传输数传信息
	Uart5_Send(Data, 22);
	//Usart2_Send(Data, 22);
}


void Can_shortDataTo32(unsigned char *Data)
{

	Data[0] = 0xAA;
	Data[1] = 0xFF;
	//Data[2] = 0x55; 			// 1 char = 1 Byte, 1 short = 2 Byte 

	Data[10] = 0xFE;
	//UT5传输数传信息
	Usart2_Send(Data, 11);
}

void Can_shortDataToK210(unsigned char *Data)
{

	DT_K210data[0] = 0xFF;
	DT_K210data[1] = 0x29; 			// 1 char = 1 Byte, 1 short = 2 Byte 
	DT_K210data[2] = Data[0];
	DT_K210data[3] = Data[1];
	DT_K210data[4] = 0xFD;
	//UT3传输数传信息
	Usart3_Send(DT_K210data, 5);
}

void Can_short_twoDataToK210(unsigned char DataA,unsigned char DataB)
{

	DT_K210data[0] = 0xFF;
	DT_K210data[1] = 0x29; 			// 1 char = 1 Byte, 1 short = 2 Byte 
	DT_K210data[2] = DataA;
	DT_K210data[3] = DataB;
	DT_K210data[4] = 0xFD;
	//UT3传输数传信息
	Usart3_Send(DT_K210data, 5);
}
u16 Ground_num= 0;
void Can_shortDataToGround(unsigned char *Data,unsigned char mode)
{
	
	if(mode == 0x03){
		Ground_num++;
	}

//	if(mode == 0x05){
//		payload_data[num_i].num =  Can_QRScan.payload;
//		payload_data[num_i].position1 = Data[5];
//		payload_data[num_i].position2 = Data[6];
//		num_i++;
//	}

	Data[0] = 0xAA;
	Data[1] = 0x29;
	Data[2] = mode;
	Data[3] = 10;//长度
	
	//Data[2] = 0x55; 			// 1 char = 1 Byte, 1 short = 2 Byte 
	
	Data[8] = Can_T265.flag;//t265flag
	Data[9] = (short)(Can_T265.x_Coordinate);
	Data[10] = (short)(Can_T265.x_Coordinate) >> 8;
	Data[11] = (short)(Can_T265.y_Coordinate);
	Data[12] = (short)(Can_T265.y_Coordinate) >> 8;
	Data[14] = 0x50;
	//UT5传输数传信息
	Usart1_Send(Data, 15);
}

void laser_blink(void){
	Can_short_twoDataToK210(0x31,0x01);//激光闪烁，亮0.5s
}

void servo_up(void){
	servo_flag = 1;//舵机在上
	Can_short_twoDataToK210(0x51,0x02);//激光闪烁，亮0.5s
}

void servo_down(void){
	servo_flag = 2;//舵机在下
	Can_short_twoDataToK210(0x51,0x03);//激光闪烁，亮0.5s
}

void servo_turn(void){
	if(servo_flag == 1){
		servo_flag = 2;
		Can_short_twoDataToK210(0x51,0x03);//激光闪烁，亮0.5s
	}else{
		servo_flag = 1;//舵机在下
		Can_short_twoDataToK210(0x51,0x02);//激光闪烁，亮0.5s
	}
	
}
