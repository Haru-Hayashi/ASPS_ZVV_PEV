#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

// *** header file include *** //
#include <stdbool.h>

// *** macro definition *** //

// *** type definition *** //
/**
 * @brief the order of LPF
 * 
 */
typedef enum{
    LPF_ZERO_ORDER = 0,
    LPF_FIRST_ORDER,
    LPF_ORDER_NUM
} LPF_ORDER;

typedef enum{
    LPF_Z0 = 0,
    LPF_Z1,
    LPF_TIME_ORDER_NUM
} LPF_TIME_ORDER;

typedef struct{
    LPF_ORDER order;
    float x[LPF_TIME_ORDER_NUM];
    float y[LPF_TIME_ORDER_NUM];
    float coef[2];
    bool enable;
} LPF_tLowPassFilter;


// *** prototype definition *** //
bool LPF_Construct(LPF_tLowPassFilter* lpf, LPF_ORDER order, float bandwidth, float Ts);
float LPF_CalcBDiff(LPF_tLowPassFilter* lpf, LPF_ORDER order, float x, float bandwidth, float Ts);
// *** prototype definition (static)*** //

#endif
