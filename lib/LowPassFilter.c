
#include "LowPassFilter.h"
#ifndef M_PI
    #define M_PI 3.14159
#endif
// *** prototype declaration (static)*** //

// *** function definition *** //
/**
 * @brief constructor of LPF
 * 
 * @param lpf object
 * @param order order of LPF
 * @param bandwidth [rad/s] bandwidth of the LPF (when this value = 0, LPF is disable)
 * @param Ts [s] sampling time
 * @return true 
 * @return false 
 */
bool LPF_Construct(LPF_tLowPassFilter* lpf, LPF_ORDER order, float bandwidth, float Ts){
    if(bandwidth==0) return 0;
    bandwidth = bandwidth / (2*M_PI);
    lpf->order = order;
    lpf->enable = 1;
    int i;
    for(i=0;i<LPF_TIME_ORDER_NUM;++i){
        lpf->x[i] = 0;
        lpf->y[i] = 0;
    }
    // if(order==LPF_ZERO_ORDER){
    //     lpf->coef[0] = 1 / (1 + Ts*bandwidth);
    //     lpf->coef[1] = (Ts*bandwidth) / (1 + Ts*bandwidth);
    // }
    return 0;
}

/**
 * @brief Calcurate the value using backward difference
 * 
 * @param lpf object
 * @param x input signal
 * @return float filtered output signal
 */
float LPF_CalcBDiff(LPF_tLowPassFilter* lpf, LPF_ORDER order, float x, float bandwidth, float Ts){
    int i;
    if(order==LPF_ZERO_ORDER){
        lpf->coef[0] = 1 / (1 + Ts*bandwidth);
        lpf->coef[1] = (Ts*bandwidth) / (1 + Ts*bandwidth);
    }
    for(i=LPF_TIME_ORDER_NUM-1;i>LPF_Z0;--i){
        lpf->x[i] = lpf->x[i-1];
        lpf->y[i] = lpf->y[i-1];
    }
    lpf->x[LPF_Z0] = x;
    if(lpf->order==LPF_ZERO_ORDER){
        lpf->y[0] = lpf->coef[0] * lpf->y[LPF_Z1] + lpf->coef[1] * lpf->x[LPF_Z0];
    }
    return lpf->y[0];
}
