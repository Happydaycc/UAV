#ifndef __CAN_DATATRANSMISSION_H__
#define __CAN_DATATRANSMISSION_H__



void Can_shortDataTransfer(unsigned char *Data, unsigned char ID);
void Can_shortDataTo32(unsigned char *Data);
void Can_shortDataToK210(unsigned char *Data);
void Can_short_twoDataToK210(unsigned char DataA,unsigned char DataB);

void Can_shortDataToGround(unsigned char *Data,unsigned char mode);
void laser_blink(void);
void servo_up(void);
void servo_down(void);
void servo_turn(void);

#endif
