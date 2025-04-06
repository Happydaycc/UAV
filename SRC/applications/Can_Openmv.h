#ifndef __CAN_OPMV_H__
#define __CAN_OPMV_H__

void OPMV_Angle_freeze(short int Target_Angle, short int Real_Ang) ;
float OPMV_Angle_PID_calculation(short int Target_Angle, short int Real_Ang);//得到PIDx速度
#endif
