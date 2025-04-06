#include "Drv_OpenMV.h"
#include "Ano_Math.h"
#include "Can_T265.h"
#include "Ano_ProgramCtrl_User.h"
#define	T265_X_KP 0.5f
#define	T265_X_KI 0.f
#define	T265_X_KD 0.15f

#define	T265_Y_KP 0.5f
#define	T265_Y_KI 0.f
#define	T265_Y_KD 0.15f

struct T265_Value
{
	float X_error;
	float X_errorLast;
	float X_errorLastL;
	float X_integral;
	float X_integralLast;
	float X_outputDelta;
	float X_output;
	
	float Y_error;
	float Y_errorLast;
	float Y_errorLastL;
	float Y_integral;
	float Y_integralLast;
	float Y_outputDelta;
	float Y_output;
};

struct T265_Value T265;

void T265_PID_Init(void)
{
	T265.X_output = 0.f;	
	T265.X_outputDelta = 0.f;
	T265.X_error = 0.f;
	T265.X_errorLast = 0.f;
	T265.X_errorLastL = 0.f; 
	T265.X_integral = 0.f;
	T265.X_integralLast = 0.f;
	
	T265.Y_output = 0.f;	
	T265.Y_outputDelta = 0.f;
	T265.Y_error = 0.f;
	T265.Y_errorLast = 0.f;
	T265.Y_errorLastL = 0.f; 
	T265.Y_integral = 0.f;
	T265.Y_integralLast = 0.f;
}

unsigned char T265_xy_goto(short int Target_dot_x, short int Target_dot_y){
	T265_x_axis_freeze(Target_dot_x, Can_T265.x_Coordinate);
	T265_y_axis_freeze(Target_dot_y, Can_T265.y_Coordinate);
	if(ABS(Target_dot_x - Can_T265.x_Coordinate) <= 5 && ABS(Target_dot_y - Can_T265.y_Coordinate) <= 5){
		//pc_user.vel_cmps_set_h[0] = 0;
		return 1;
	}
	else return 0;
}

short int f4_Target_dot_y;
unsigned char T265_xy_goto2(short int Target_dot_x, short int Target_dot_y){
	f4_Target_dot_y = Target_dot_y;
	T265_x_axis_freeze2(Target_dot_x, Can_T265.x_Coordinate);
	T265_y_axis_freeze2(Target_dot_y, Can_T265.y_Coordinate);
	if(ABS(Target_dot_x - Can_T265.x_Coordinate) <= 5 && ABS(Target_dot_y - Can_T265.y_Coordinate) <= 5){
		//pc_user.vel_cmps_set_h[0] = 0;
		return 1;
	}
	else{return 0;}
}


/*
����;Ŀ������뵱ǰλ��
*/
char T265_x_flag = 1;
short int T265_LastDot_X = 0;
float T265_x_axis_PID_calculation(short int Target_dot_x, short int Dot_X)//�õ�PIDx�ٶ�
{	
	T265_LastDot_X = Dot_X;
			
	if (T265_x_flag == 1)
	{
		T265_PID_Init();
		T265_x_flag = 0;
	}
	else
	{
		T265.X_outputDelta = T265_X_KP * (T265.X_error - T265.X_errorLast)+
		T265_X_KI * (T265.X_integral - T265.X_integralLast) +
		T265_X_KD * (T265.X_error - 2 * T265.X_errorLast + T265.X_errorLastL);

		T265.X_errorLastL = T265.X_errorLast;
		T265.X_errorLast = T265.X_error;

		T265.X_output += T265.X_outputDelta;
		T265.X_error = Target_dot_x - Dot_X; 
		T265.X_integralLast = T265.X_integral;
		T265.X_integral += T265.X_error;
	}		

	return T265.X_output;				
}

char T265_y_flag = 1;
short int T265_LastDot_Y = 0;
float T265_y_axis_PID_calculation(short int Target_dot_y, short int Dot_Y)//�õ�PIDy�ٶ�
{	
	T265_LastDot_Y = Dot_Y;
			
	if (T265_y_flag == 1)
	{
		T265_PID_Init();
		T265_y_flag = 0;
	}
	else
	{
		T265.Y_outputDelta = T265_Y_KP * (T265.Y_error - T265.Y_errorLast)+
		T265_Y_KI * (T265.Y_integral - T265.Y_integralLast) +
		T265_Y_KD * (T265.Y_error - 2 * T265.Y_errorLast + T265.Y_errorLastL);

		T265.Y_errorLastL = T265.Y_errorLast;
		T265.Y_errorLast = T265.Y_error;

		T265.Y_output += T265.Y_outputDelta;
		T265.Y_error = Target_dot_y - Dot_Y; 
		T265.Y_integralLast = T265.Y_integral;
		T265.Y_integral += T265.Y_error;
	}		

	return T265.Y_output;				
}

float x_coordinate_when_init = 0.f;
void T265_x_init(void)
{
	x_coordinate_when_init = Can_T265.x_Coordinate;
}

float y_coordinate_when_init = 0.f;
void T265_y_init(void)
{
	y_coordinate_when_init = Can_T265.y_Coordinate;
}

// ��������freezeʹ��֮ǰ����ҪT265_x_initһ�£�Ȼ��x_coordinate_when_init��Ϊ�βθ�����
float T265_x_speed;
void T265_x_axis_freeze(short Target_dot_x, short Dot_X) 
{
	T265_x_speed = T265_x_axis_PID_calculation(Target_dot_x, Dot_X);
	
	if (ABS(T265_x_speed) <= 0.5f)
	{
		T265_x_speed = 0;
	}
	
	T265_x_speed = LIMIT(T265_x_speed, -15, 15);
	pc_user.vel_cmps_set_h[0] = T265_x_speed;			
}

float T265_y_speed;
void T265_y_axis_freeze(short Target_dot_y, short Dot_Y) 
{
	T265_y_speed = T265_y_axis_PID_calculation(Target_dot_y, Dot_Y);
	
	if (ABS(T265_y_speed) <= 0.5f)
	{
		T265_y_speed = 0;
	}
	
	T265_y_speed = LIMIT(T265_y_speed, -15, 15);
	pc_user.vel_cmps_set_h[1] = T265_y_speed;			
}

float T265_x_speed2;
void T265_x_axis_freeze2(short Target_dot_x, short Dot_X) 
{
	T265_x_speed2 = T265_x_axis_PID_calculation(Target_dot_x, Dot_X);
	
	if (ABS(T265_x_speed2) <= 0.5f)
	{
		T265_x_speed2 = 0;
	}
	
	T265_x_speed2 = LIMIT(T265_x_speed2, -15, 15);
	pc_user.vel_cmps_set_h[0] = -T265_x_speed2;			
}

float T265_y_speed2;
void T265_y_axis_freeze2(short Target_dot_y, short Dot_Y) 
{
	T265_y_speed2 = T265_y_axis_PID_calculation(Target_dot_y, Dot_Y);
	
	if (ABS(T265_y_speed2) <= 0.5f)
	{
		T265_y_speed2 = 0;
	}
	
	T265_y_speed2 = LIMIT(T265_y_speed2, -15, 15);
	pc_user.vel_cmps_set_h[1] = -T265_y_speed2;			
}



void T265_xy_axis(short x, short y){
	T265_x_axis_freeze( x,  Can_T265.x_Coordinate) ;
	T265_y_axis_freeze( y,  Can_T265.y_Coordinate);
} 

void T265_xy_axis2(short x, short y){
	T265_x_axis_freeze2( x,  Can_T265.x_Coordinate) ;
	T265_y_axis_freeze2( y,  Can_T265.y_Coordinate);
} 

//������
/*
��������T265_x_axis_arrive_cm
����˵��������t265ʵ�ַɻ�����ϵ�£�x�᷽��ľ�ȷ�ƶ�������cm
������x_axis_speed, x_axis_cm, x_axis_flag
x_axis__speed��ʾx�᷽����ٶ�
x_axis_cm��ʾx�᷽���ƶ��ľ���
����ֵ����
���ڣ�2023��7��25��
*/
unsigned char T265_x_init_flag = 1;
unsigned char T265_x_axis_arrive_cm(float x_axis_speed, float x_axis_cm)
{
	static unsigned char T265_x_taskconfirmcount = 0;
	unsigned char T265_x_success_flag = 0;
	if (T265_x_init_flag == 1)
	{
		T265_x_init();
		T265_y_init();
		T265_x_init_flag = 0;
	}
	
	T265_y_axis_freeze(y_coordinate_when_init, Can_T265.y_Coordinate);	// y���ֹ�ƶ�
	x_axis_speed = LIMIT(x_axis_speed, -30, 30);
	pc_user.vel_cmps_set_h[0] = x_axis_speed;
	
	if (ABS(Can_T265.x_Coordinate - x_coordinate_when_init) >= ABS(x_axis_cm))
	{
		if (T265_x_taskconfirmcount <= 10)
		{
			T265_x_taskconfirmcount	+= 1;
		}
		else
		{
			T265_x_taskconfirmcount = 0;
			T265_x_init_flag = 1;
			T265_x_success_flag = 1;
		}
	}
	
	return T265_x_success_flag;
}


/*
��������T265_y_axis_arrive_cm
����˵��������t265ʵ�ַɻ�����ϵ�£�y�᷽��ľ�ȷ�ƶ�������cm
������y_axis_speed, y_axis_cm, y_axis_flag
y_axis__speed��ʾy�᷽����ٶ�
y_axis_cm��ʾy�᷽���ƶ��ľ���
����ֵ����
���ڣ�2023��7��25��
*/
unsigned char T265_y_init_flag = 1;
unsigned char T265_y_axis_arrive_cm(float y_axis_speed, float y_axis_cm)
{
	static unsigned char T265_y_taskconfirmcount = 0;
	unsigned char T265_y_success_flag = 0;
	if (T265_y_init_flag == 1)
	{
		T265_x_init();
		T265_y_init();
		T265_y_init_flag = 0;
	}
	
	T265_x_axis_freeze(x_coordinate_when_init, Can_T265.y_Coordinate);	// x���ֹ�ƶ�
	
	y_axis_speed = LIMIT(y_axis_speed, -30, 30);
	pc_user.vel_cmps_set_h[1] = y_axis_speed;
	
	if (ABS(Can_T265.y_Coordinate - y_coordinate_when_init) >= ABS(y_axis_cm))
	{
		if (T265_y_taskconfirmcount <= 10)
		{
			T265_y_taskconfirmcount	+= 1;
		}
		else
		{
			T265_y_taskconfirmcount = 0;
			T265_y_init_flag = 1;
			T265_y_success_flag = 1;
		}
	}
	
	return T265_y_success_flag;
}

// 0��ʾδ����1��ʾ�ѵ������Զ��ж���һ��x_axis_cm
unsigned char T265_x_detector_init_flag = 1;
float X_detector_when_init = 0.f;
unsigned char T265_x_axis_detector_cm(float x_axis_cm)//�ж��ƶ������Ƿ񵽴�
{
	static unsigned char T265_x_taskconfirmcount;
	unsigned char T265_x_taskconfirmflag = 0;
	
	if (T265_x_detector_init_flag == 1)
	{
		X_detector_when_init = Can_T265.x_Coordinate;
		T265_x_detector_init_flag = 0;
	}
	
	if (ABS(Can_T265.x_Coordinate - X_detector_when_init) >= ABS(x_axis_cm))
	{
		if (T265_x_taskconfirmcount <= 10)
		{
			T265_x_taskconfirmcount += 1;
		}
		else
		{
			T265_x_taskconfirmcount = 0;
			T265_x_detector_init_flag = 1;
			T265_x_taskconfirmflag = 1;
		}
	}
	
	return T265_x_taskconfirmflag;
}

unsigned char T265_y_detector_init_flag = 1;
float Y_detector_when_init = 0.f;
unsigned char T265_y_axis_detector_cm(float y_axis_cm)
{
	static unsigned char T265_y_taskconfirmcount;
	unsigned char T265_y_taskconfirmflag = 0;
	
	if (T265_y_detector_init_flag == 1)
	{
		Y_detector_when_init = Can_T265.y_Coordinate;
		T265_y_detector_init_flag = 0;
	}
	
	if (ABS(Can_T265.y_Coordinate - Y_detector_when_init) >= ABS(y_axis_cm))
	{
		if (T265_y_taskconfirmcount <= 10)
		{
			T265_y_taskconfirmcount += 1;
		}
		else
		{
			T265_y_taskconfirmcount = 0;
			T265_y_detector_init_flag = 1;
			T265_y_taskconfirmflag = 1;
		}
	}
	
	return T265_y_taskconfirmflag;
}
