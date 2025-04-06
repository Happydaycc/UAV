#ifndef __CAN_T265_H__
#define __CAN_T265_H__

void T265_x_axis_freeze(short Target_dot_x, short Dot_X);
void T265_y_axis_freeze(short Target_dot_y, short Dot_Y);

void T265_x_axis_freeze2(short Target_dot_x, short Dot_X);
void T265_y_axis_freeze2(short Target_dot_y, short Dot_Y);

void T265_xy_axis(short x, short y);
void T265_xy_axis2(short x, short y);

unsigned char T265_xy_goto(short int Target_dot_x, short int Target_dot_y);
unsigned char T265_xy_goto2(short int Target_dot_x, short int Target_dot_y);

#endif
