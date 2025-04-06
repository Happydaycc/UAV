#ifndef __SHELVES_H__
#define __SHELVES_H__

#include "Ano_FcData.h"

typedef struct
{
	s16 order;//序号
	u8 flag;	//是否检测到
	u8 num;		//二维码的数字
	s16 x_Coordinate;
	s16 y_Coordinate;
}Goods;

extern Goods goods[30];

#endif
