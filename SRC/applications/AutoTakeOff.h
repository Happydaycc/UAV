#ifndef _AUTOTAKEOFF_H_
#define _AUTOTAKEOFF_H_


void Remote_Control(unsigned char dT_ms);
void KeepHeight(unsigned int Height);
void AutoTakeOff(unsigned char dT_ms);
void my_PID_Init(double Target);
double PID_Calculation(double Target);
void PID_Init(void);

#define Yes 1
#define No 0

#define OFF_CM 125.f

#define RELATIVE_HEIGHT_CM  (jsdata.valid_of_alt_cm)
#define AUTOTAKEOFFSPEEDMAX 20
#define KEEPHEIGHTSPEEDMAX 20


#endif
