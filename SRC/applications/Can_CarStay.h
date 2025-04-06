#ifndef __CAN_DOTSTAY_H__
#define __CAN_DOTSTAY_H__

void K210_Stay(unsigned char Dot_Speical_Point, short int Dot_X, short int Dot_Y, unsigned int KeepHeight_cm);
short int K210DataProcess(short int Dot_Position, unsigned int KeepHeight_cm);

#endif
