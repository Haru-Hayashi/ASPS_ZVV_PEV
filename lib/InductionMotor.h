#ifndef INDUCTION_MOTOR_H_
#define INDUCTION_MOTOR_H_

// *** header file include *** //
#include <stdbool.h>
#include "Common.h"
#include "Vector.h"
#include "../UserDefined.h"

// *** macro definition *** //

// *** type definition *** //
// IM Parameter
typedef struct{
    float L[2];
    float R[2];
    float M;
    float Kt;
    float p;
    float Jm;
    float Dm;
    float sigma;
    float Lsigma;
} IM_tMotorParam;

/**
 * @brief 状態量まとめ
 * 
 * @param ires 電流応答
 * @param vres 電圧応答
 * @param phi1 一次鎖交磁束応答
 * @param phi2 二次鎖交磁束応答
 * @param torque トルク応答
 * @param theta_1   一次位相
 * @param theta_2   二次位相
 * @param omega_1   一次角周波数
 * @param omega_2   二次角周波数
 * @param omega_s   すべり角周波数
 * @param omega_re  電気角
 * @param omega_rm  機械角
 * @param omega_rm_lpf  エンコーダ取得用LPF
 * 
 * @param Ts    サンプリング周期
 * 
 * @param omega_s_coef  すべり角周波数演算のための係数
 * @param phi2dq_coef[3]    dq軸2次鎖交磁束演算のための係数
 */
typedef struct{
    VCT_tVect3State ires;
    VCT_tVect3State vres;
    VCT_tVect2State phi1;
    VCT_tVect2State phi2;
    float torque;
    float theta_1;
    float theta_2;
    float omega_1;
    float omega_2;
    float omega_s;
    float omega_re;
    float omega_rm;
    float omega_rm_lpf;
    float Ts;

    float omega_s_coef;
    float phi2dq_coef[3];
} IM_tMotorState;

// IM
typedef struct{
    unsigned int id;
    IM_tMotorParam mp;
    IM_tMotorState ms;
}IM_tMotor;


// *** prototype declaration *** //
bool IM_Construct(IM_tMotor* m, float* IM_ParamArray, unsigned int ParamArraySize, float Ts);
bool IM_InitParam(IM_tMotorParam* mp, float* IM_ParamArray, unsigned int ParamArraySize);
bool IM_InitState(IM_tMotorState* ms, float Ts);

bool IM_CalcStatus(IM_tMotor* m, float Tcalc);

#endif
