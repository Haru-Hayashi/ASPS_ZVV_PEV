#ifndef AXISCONV_H
#define AXISCONV_H

#include "Vector.h"

// *** 列挙型 *** //
typedef enum{
    AXIS_UVW = 0,
    AXIS_AB,
    AXIS_DQ,
    AXIS_NUM
} AXIS;

void Vect3AxisConv(tVect3State* vs, AXIS org, AXIS cnv); //任意の座標変換
void Vect2AxisConv(tVect2State* vs, AXIS org, AXIS cnv);

#endif

