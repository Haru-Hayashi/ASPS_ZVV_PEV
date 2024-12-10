#ifndef COMMOM_H
#define COMMOM_H

// ***** header file include ***** //
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

// *** prototype declaration (static)*** //

// *** function definition *** //


// *** prototype declaration (static)*** //

// *** function definition *** //

// ***** macro definition ***** //

// *** 変数 *** //
// 円周率
#define M_PI (3.14159265358979)
// 2/3*pi
#define PI23 (2.094395102393195)
// sqrt(2/3)
#define SQ23 (0.816496580927726)
// sqrt(3/2)
#define SQ32 (1.224744871391589)

// *** マクロ関数 *** //
// PI(n)でnπ 例：PI(2) => 2*π
#define PI(n)   ((float)M_PI*(n))
// リミッタ
#define	LIMIT(in,limit) (((in)>(limit))?(limit):(((in)<(-(limit)))?(-(limit)):(in)))
// 直交座標(x, y)からノルムを求める
#define	SQRT2(x,y)  (sqrt((x)*(x)+(y)*(y)))
// 符号関数
#define SGN(x)  (((x)>(0.0))?(1.0):(((x)<(0.0))?(-1.0):(0.0)))
// 絶対値
#define ABS(x)  (((x)>=(0)?(x):(-(x))))
// 四捨五入
#define RND(x)  ((int)((x)+0.5))
// 最大・最小（_INDEXマクロの戻り値は引数での順番を返すので注意）
#define MAX(val,maximum)    (((val)>(maximum))?(val):(maximum))
#define MIN(val,minimum)    (((val)<(minimum))?(val):(minimum))
#define MAX2_INDEX(num0,num1)          (((num0)>(num1))?(0):(1))
#define MAX3_INDEX(num0,num1, num2)    (((num0)>(num1))?(((num2)>(num0))?(2):(0)):(((num2)>(num1))?(2):(1)))
#define MIN2_INDEX(num0,num1)          (((num0)<(num1))?(0):(1))
#define MIN3_INDEX(num0,num1, num2)    (((num0)<(num1))?(((num2)<(num0))?(2):(0)):(((num2)<(num1))?(2):(1)))
// 分解能の変更
#define RESAMPLE(var,rsl)   (((int)((var)/(rsl)))*(rsl))
// 位相範囲の変更
#define CNV_THETA(phase)    (((phase)<0)?((phase)+PI(2)):(phase))

// ***** type definition ***** //
typedef struct{
    float value;
    int index;
}COM_tIndex;


int COM_Max(float* arr, uint32_t numlen);
int COM_Max2(float num1, float num2);
int COM_Max3(float num1, float num2, float num3);
int COM_Min(float* arr, uint32_t numlen);
int COM_Min2(float num1, float num2);
int COM_Min3(float num1, float num2, float num3);

void COM_MinIndex(float* value, int num, COM_tIndex* ret);
float COM_Limiter(float sig, float lim_val, float* delta);

int COM_GetOrderFromIndex(COM_tIndex* ind, int order);

// ***** prototype definition ***** //


#endif