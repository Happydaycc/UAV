//默认引用：
#include "Drv_OpenMV.h"

//设定
#define OPMV_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
u16 offline_check_time;
u8 openmv_buf[20];
u8 T265_buf[20];
u8 Ground_buf[20];
_openmv_data_st opmv;

_Can_opmv_data_st Can_opmv;//自定义opmv
_Can_K210_data_st Can_K210;//自定义K210
_Can_T265_data_st Can_T265;//T265
_Can_QR_data_st Can_QRScan;//自定义扫码器
/**********************************************************************************************************
*函 数 名: T265_Byte_Get
*功能说明: T265字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void T265_Byte_Get(u8 bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	T265_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(1)//(bytedata==0x29)未确定
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(bytedata==0x43)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<20)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += T265_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			T265_Data_Analysis(T265_buf,len+6);
			
			rec_sta=0;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else
	{
		//	
		rec_sta++;
	}
}
/**********************************************************************************************************
*函 数 名: T265_Data_Analysis
*功能说明: T265数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
#include "Ano_DT.h"
static void T265_Data_Analysis(u8 *buf_data,u8 len)
{
	//0x41为K210标志位，0x42为OpenMV标志位
	if(*(buf_data+3)==0x43)
	{
		Can_T265.x_Coordinate = -(s16)((*(buf_data+6)<<8)|*(buf_data+5));//x和y根据实际情况调整
		Can_T265.y_Coordinate = (s16)((*(buf_data+8)<<8)|*(buf_data+7));
		Can_T265.z_Coordinate = (s16)((*(buf_data+10)<<8)|*(buf_data+9));
		Can_T265.flag = 1;
		opmv.mode_sta = 3;
	}
	OpenMV_Check_Reset();
}
/**********************************************************************************************************
*函 数 名: OpenMV_Byte_Get
*功能说明: OpenMV字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void OpenMV_Byte_Get(u8 bytedata)
{	
	static u8 len = 0,rec_sta;
	u8 check_val=0;
	
	//
	openmv_buf[rec_sta] = bytedata;
	//
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(1)//(bytedata==0x29)未确定
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(bytedata==0x05)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(bytedata==0x41 || bytedata==0x42)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		//
		len = bytedata;
		if(len<20)
		{
			rec_sta++;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==(len+5))
	{
		//
		for(u8 i=0;i<len+5;i++)
		{
			check_val += openmv_buf[i];
		}
		//
		if(check_val == bytedata)
		{
			//解析成功
			OpenMV_Data_Analysis(openmv_buf,len+6);
			//
			rec_sta=0;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else
	{
		//	
		rec_sta++;
	}
}

/**********************************************************************************************************
*函 数 名: OpenMV_Data_Analysis
*功能说明: OpenMV数据解析
*参    数: 缓存数据（形参），长度
*返 回 值: 无
**********************************************************************************************************/
#include "Ano_DT.h"
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len)
{
	
	//0x41为K210标志位，0x42为OpenMV标志位
	if(*(buf_data+3)==0x41)
	{
//		Can_K210.flag = *(buf_data+6); 
//		Can_K210.y = (s16)((*(buf_data+7)<<8)|*(buf_data+8));//x和y根据实际情况调整
//		Can_K210.x = (s16)((*(buf_data+9)<<8)|*(buf_data+10));
		Can_QRScan.flag = *(buf_data+6);
		Can_QRScan.payload = *(buf_data+7);
		Can_QRScan.last_payload = Can_QRScan.payload;
	}
	else if(*(buf_data+3)==0x42)
	{
//		opmv.lt.sta = *(buf_data+5);
//		opmv.lt.angle = (s16)((*(buf_data+6)<<8)|*(buf_data+7));
//		opmv.lt.deviation = (s16)((*(buf_data+8)<<8)|*(buf_data+9));
//		opmv.lt.p_flag = *(buf_data+10);
//		opmv.lt.pos_x = (s16)((*(buf_data+11)<<8)|*(buf_data+12));
//		opmv.lt.pos_y = (s16)((*(buf_data+13)<<8)|*(buf_data+14));
//		opmv.lt.dT_ms = *(buf_data+15);
		Can_opmv.flag = *(buf_data+6);
		Can_opmv.payload = (s16)((*(buf_data+7)<<8)|*(buf_data+8));	
		opmv.mode_sta = 1;
	}
}

/**********************************************************************************************************
*函 数 名: OpenMV_Offline_Check
*功能说明: OpenMV掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void OpenMV_Offline_Check(u8 dT_ms)
{
	if(offline_check_time<OPMV_OFFLINE_TIME_MS)
	{
		offline_check_time += dT_ms;
	}
	else
	{
		opmv.offline = 1;
		opmv.mode_sta = 0;
	}
	
}

/**********************************************************************************************************
*函 数 名: OpenMV_Check_Reset
*功能说明: OpenMV掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void OpenMV_Check_Reset()
{
	offline_check_time = 0;
	opmv.offline = 0;
}

/**********************************************************************************************************
*函 数 名: Ground_Byte_Get
*功能说明: 地面站字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
void Ground_Byte_Get(u8 bytedata)
{	
	static u8 rec_sta;
	Ground_buf[rec_sta] = bytedata;
	if(rec_sta==0)
	{
		if(bytedata==0xaa)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(bytedata==0x29) //未确定
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(1)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		if(1)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==4)
	{
		if(bytedata==0x50)
		{
			Ground_Data_Analysis(Ground_buf);
			//Can_shortDataToGround(Ground_data,0x05);
			rec_sta=0;
		}		
		else
		{
			rec_sta=0;
		}
	}
	else
	{
		//	
		rec_sta++;
	}
}

static void Ground_Data_Analysis(u8 *buf_data)
{
	//0x41为K210标志位，0x42为OpenMV标志位
//	if(*(buf_data+3)==0x43)
//	{
//		Can_T265.x_Coordinate = -(s16)((*(buf_data+6)<<8)|*(buf_data+5));//x和y根据实际情况调整
//		Can_T265.flag = 1;
//	}
	Can_QRScan.Secondtask_position1 = *(buf_data+2);
	Can_QRScan.Secondtask_position2 = *(buf_data+3) - '0';
	Can_QRScan.Secondtask_flag = 1;
}

