#ifndef __SHELVES_H__
#define __SHELVES_H__

#include "Ano_FcData.h"

typedef struct
{
	s16 order;//���
	u8 flag;	//�Ƿ��⵽
	u8 num;		//��ά�������
	s16 x_Coordinate;
	s16 y_Coordinate;
}Goods;

extern Goods goods[30];

#endif
