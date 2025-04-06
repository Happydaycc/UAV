//Ĭ�����ã�
#include "Drv_OpenMV.h"

//�趨
#define OPMV_OFFLINE_TIME_MS  1000  //����

//ȫ�ֱ���
u16 offline_check_time;
u8 openmv_buf[20];
u8 T265_buf[20];
u8 Ground_buf[20];
_openmv_data_st opmv;

_Can_opmv_data_st Can_opmv;//�Զ���opmv
_Can_K210_data_st Can_K210;//�Զ���K210
_Can_T265_data_st Can_T265;//T265
_Can_QR_data_st Can_QRScan;//�Զ���ɨ����
/**********************************************************************************************************
*�� �� ��: T265_Byte_Get
*����˵��: T265�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
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
		if(1)//(bytedata==0x29)δȷ��
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
			//�����ɹ�
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
*�� �� ��: T265_Data_Analysis
*����˵��: T265���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
#include "Ano_DT.h"
static void T265_Data_Analysis(u8 *buf_data,u8 len)
{
	//0x41ΪK210��־λ��0x42ΪOpenMV��־λ
	if(*(buf_data+3)==0x43)
	{
		Can_T265.x_Coordinate = -(s16)((*(buf_data+6)<<8)|*(buf_data+5));//x��y����ʵ���������
		Can_T265.y_Coordinate = (s16)((*(buf_data+8)<<8)|*(buf_data+7));
		Can_T265.z_Coordinate = (s16)((*(buf_data+10)<<8)|*(buf_data+9));
		Can_T265.flag = 1;
		opmv.mode_sta = 3;
	}
	OpenMV_Check_Reset();
}
/**********************************************************************************************************
*�� �� ��: OpenMV_Byte_Get
*����˵��: OpenMV�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
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
		if(1)//(bytedata==0x29)δȷ��
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
			//�����ɹ�
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
*�� �� ��: OpenMV_Data_Analysis
*����˵��: OpenMV���ݽ���
*��    ��: �������ݣ��βΣ�������
*�� �� ֵ: ��
**********************************************************************************************************/
#include "Ano_DT.h"
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len)
{
	
	//0x41ΪK210��־λ��0x42ΪOpenMV��־λ
	if(*(buf_data+3)==0x41)
	{
//		Can_K210.flag = *(buf_data+6); 
//		Can_K210.y = (s16)((*(buf_data+7)<<8)|*(buf_data+8));//x��y����ʵ���������
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
*�� �� ��: OpenMV_Offline_Check
*����˵��: OpenMV���߼�⣬�������Ӳ���Ƿ�����
*��    ��: ʱ�䣨���룩
*�� �� ֵ: ��
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
*�� �� ��: OpenMV_Check_Reset
*����˵��: OpenMV���߼�⸴λ��֤��û�е���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OpenMV_Check_Reset()
{
	offline_check_time = 0;
	opmv.offline = 0;
}

/**********************************************************************************************************
*�� �� ��: Ground_Byte_Get
*����˵��: ����վ�ֽ����ݻ�ȡ
*��    ��: �ֽ�����
*�� �� ֵ: ��
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
		if(bytedata==0x29) //δȷ��
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
	//0x41ΪK210��־λ��0x42ΪOpenMV��־λ
//	if(*(buf_data+3)==0x43)
//	{
//		Can_T265.x_Coordinate = -(s16)((*(buf_data+6)<<8)|*(buf_data+5));//x��y����ʵ���������
//		Can_T265.flag = 1;
//	}
	Can_QRScan.Secondtask_position1 = *(buf_data+2);
	Can_QRScan.Secondtask_position2 = *(buf_data+3) - '0';
	Can_QRScan.Secondtask_flag = 1;
}

