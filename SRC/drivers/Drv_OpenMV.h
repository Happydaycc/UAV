#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==引用

#include "Ano_FcData.h"

//==定义
typedef struct
{
	//
	u8 color_flag;
	u8 sta;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 deviation;
	u8 p_flag;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_line_tracking_st;

typedef struct
{
	u8 offline;
	u8 mode_cmd;
	u8 mode_sta;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
}_openmv_data_st;
//==数据声明
extern _openmv_data_st opmv;


//==自定义 + 数据声明
typedef struct
{
	u8 flag;
	s16 angle;
	s16 distance;
	u8 cross_flag;
	s16 cross_x;
	s16 cross_y;
	u16 payload;
}_Can_opmv_data_st;
extern _Can_opmv_data_st Can_opmv;
typedef struct
{
	u8 flag;
	u8 crossflag;//直线交点标志
	s16 angle;
	s16 distance;
	s16 x;
	s16 y;
}_Can_K210_data_st;
extern _Can_K210_data_st Can_K210;

typedef struct
{
	s16 x_Coordinate;//x坐标
	s16 y_Coordinate;
	s16 z_Coordinate;
	u8 flag;
}_Can_T265_data_st;
extern _Can_T265_data_st Can_T265;

typedef struct
{
	u8 payload;//二维码扫到的数字
	u8 last_payload;
	u8 Secondtask_payload;
	u8 Secondtask_position1;	//数字所在位置编号第一位
	u8 Secondtask_position2;	//数字所在位置编号第二位
	u8 Secondtask_flag;
	u8 flag;
}_Can_QR_data_st;
extern _Can_QR_data_st Can_QRScan;
//==函数声明

//static
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);
void OpenMV_Byte_Get(u8 bytedata);

//T265
void T265_Byte_Get(u8 bytedata);
static void T265_Data_Analysis(u8 *buf_data,u8 len);

//Ground
void Ground_Byte_Get(u8 bytedata);
static void Ground_Data_Analysis(u8 *buf_data);
#endif

