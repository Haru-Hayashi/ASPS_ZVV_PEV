
#include "InductionMotor.h"

// *** prototype declaration (static)*** //
static void CalcEstimationCoef(IM_tMotorParam* mp, IM_tMotorState* ms, float Ts);
static float IM_CalcPhi2d(IM_tMotorState* ms);
static float IM_CalcOmegaS(IM_tMotorState* ms);
static float IM_CalcOmegaRe(IM_tMotorState* ms, float p);
static float IM_CalcOmega1(IM_tMotorState* ms);
static float IM_CalcPhi2Phase(IM_tMotorState* ms, float Tcalc);

// *** variable *** //
IM_tMotor *pm;

// *** function definition *** //

/**
 * @brief parameter initialization of IM
 * 
 * @param mp initialized parameter
 * @param IM_ParamArray parameters
 * @param R1 [ohm] stator resistor (dq)
 * @param R2 [ohm] rotor resistor (dq)
 * @param L1 [H] stator inductance (dq)
 * @param L2 [H] rotor inductance (dq)
 * @param M  [H] mutual inductance (dq)
 * @param p [-] pole pair (dq)
 * @param Jm [Nm/(rad/s^2)] motor inertia
 * @param Dm [Nm/(rad/s)] motor viscouse friction
 * @param ParamArraySize size of parameters array (8 is correctly)
 * @return true success
 * @return false error
 */
bool IM_InitParam(IM_tMotorParam* mp, float* IM_ParamArray, unsigned int ParamArraySize){
    if(ParamArraySize!=8) return -1;
    else{
        mp->R[0]    = IM_ParamArray[0];
        mp->R[1]    = IM_ParamArray[1];
        mp->L[0]    = IM_ParamArray[2];
        mp->L[1]    = IM_ParamArray[3];
        mp->M       = IM_ParamArray[4];
        mp->p       = IM_ParamArray[5];
        mp->Jm      = IM_ParamArray[6];
        mp->Dm      = IM_ParamArray[7];
        mp->sigma = 1 - mp->M*mp->M/(mp->L[0]*mp->L[1]);
        mp->Lsigma = mp->L[0] - mp->M * mp->M / mp->L[1];
    }
    return 0;
}

/**
 * @brief State initializetion of IM
 * 
 * @param ms initialized state
 * @param Ts sampling time [s]
 * @return true 
 * @return false 
 */
bool IM_InitState(IM_tMotorState* ms, float Ts){
    InitVect3(&(ms->vres));
    InitVect3(&(ms->ires));
    InitVect2(&(ms->phi1));
    InitVect2(&(ms->phi2));
    ms->omega_1 = 0;
    ms->omega_2 = 0;
    ms->omega_s = 0;
    ms->omega_re = 0;
    ms->theta_1 = 0;
    ms->theta_2 = 0;
    ms->torque = 0;

    ms->Ts = Ts;

    return 0;
}

/**
 * @brief constructor of IM
 * 
 * @param m IM's structure
 * @param IM_ParamArray parameters
 * @param ParamArraySize size of parameters
 * @param Ts sampling time
 * @return true 
 * @return false 
 */
bool IM_Construct(IM_tMotor* m, float* IM_ParamArray, unsigned int ParamArraySize, float Ts){
    if(IM_InitParam(&(m->mp), IM_ParamArray, ParamArraySize)!=0) return -1;
    IM_InitState(&(m->ms), Ts);
    CalcEstimationCoef(&(m->mp), &(m->ms), Ts);
    return 0;
}


// ***** 状態量の演算 ***** //
/**
 * @brief dq軸電流を得るための二次鎖交磁束位相を演算
 *        そのために必要な各状態量演算
 * 
 * @param ms モータ状態量
 * @param Tcalc 演算周期
 * @return true 
 * @return false 
 */
bool IM_CalcStatus(IM_tMotor* m, float Tcalc){
    // dq軸2次鎖交磁束（FOC前提のため、q軸は0）
    IM_CalcPhi2d(&(m->ms));
    // すべり角周波数
    IM_CalcOmegaS(&(m->ms));
    // 電気角速度
    IM_CalcOmegaRe(&(m->ms), m->mp.p);
    // 1次角周波数
    IM_CalcOmega1(&(m->ms));
    // 2次鎖交磁束位相
    IM_CalcPhi2Phase(&(m->ms), Tcalc);
    
    // 2次鎖交磁束の大きさ
    return 0;
}

// ******* static ******* //

/**
 * @brief calcuration coefficients for estimation
 * 
 * @param mp motor param structure
 * @param ms motor state structure
 * @param Ts sampling time
 */
static void CalcEstimationCoef(IM_tMotorParam* mp, IM_tMotorState* ms, float Ts){
    // *** すべり角周波数　推定用 *** //
    ms->omega_s_coef = mp->M*mp->R[1]/mp->L[1];
    // *** FOC適応前提の二次鎖交磁束推定用係数（後退差分） *** //
    // d軸
    ms->phi2dq_coef[0] = 1.0/(1.0+mp->R[1]/mp->L[1]*Ts);
    ms->phi2dq_coef[1] = mp->M*mp->R[1]/mp->L[1]*Ts;
    // q軸
    ms->phi2dq_coef[2] = mp->M * mp->R[1] / mp->L[1];
    return;
}

/**
 * @brief   dq軸二次鎖交磁束の演算
 * 
 * @param m モータポインタ
 * @return float d軸二次鎖交磁束[Wb]（q軸は0）
 */
static float IM_CalcPhi2d(IM_tMotorState* ms){
    // d軸:下記if文において==0は計算誤差含めありえないが、I1_EPSILONを使うと磁束応答が反転してあらわれることがある（q軸電流も負でてるためトルクは同じ，制御を入れて指令値を与えると問題なし，要調査）
    if(mwabs(ms->ires.dq[0])<I1_EPSILON) ms->phi2.dq[0] = 0;
//    if(mwabs(ms->ires.dq[0])==0) ms->phi2.dq[0] = 0;
    else ms->phi2.dq[0] = ms->phi2dq_coef[0] * (ms->phi2dq_coef[1] * ms->ires.dq[0] + ms->phi2.dq[0]);
    // q軸
    ms->phi2.dq[1] = ms->phi2dq_coef[2] * ms->ires.dq[1] - ms->phi2.dq[0] * ms->omega_s;
    // 大きさ
    ms->phi2.nrm = SQRT2(ms->phi2.dq[0], ms->phi2.dq[1]);
    return ms->phi2.dq[0];
}

/**
 * @brief すべり角周波数の演算
 * 
 * @param m モータポインタ
 * @return float すべり角周波数[rad/s]
 */
static float IM_CalcOmegaS(IM_tMotorState* ms){
    // 下記if文において==0は計算誤差含めありえないが、I1_EPSILONを使うと磁束応答が反転してあらわれることがある（q軸電流も負でてるためトルクは同じ，制御を入れて指令値を与えると問題なし，要調査）
    if(mwabs(ms->phi2.dq[0])<EPSILON) ms->omega_s = 0;
//    if(mwabs(ms->phi2.dq[0])==0) ms->omega_s = 0;
    else ms->omega_s = ms->omega_s_coef * ms->ires.dq[1] / ms->phi2.dq[0];
    return ms->omega_s;
}

/**
 * @brief 電気角速度の計算
 * 
 * @param m モータポインタ
 * @return float 電気角周波数[rad/s]
 */
static float IM_CalcOmegaRe(IM_tMotorState* ms, float p){
    ms->omega_re = ms->omega_rm * p;
    return ms->omega_re;
}

/**
 * @brief 一次角速度の計算
 * 
 * @param m モータポインタ
 * @return float 一次角周波数[rad/s]
 */
static float IM_CalcOmega1(IM_tMotorState* ms){
    ms->omega_1 = ms->omega_re + ms->omega_s;
    return ms->omega_1;
}

/**
 * @brief 二次鎖交磁束位相の演算
 * 
 * @param m モータポインタ
 * @param Tcalc 演算周期
 * @return float 二次鎖交磁束位相
 */
static float IM_CalcPhi2Phase(IM_tMotorState* ms, float Tcalc){
    ms->phi2.phase += ms->omega_1 * Tcalc;
    while(ms->phi2.phase>PI(1)) ms->phi2.phase -= PI(2);
    while(ms->phi2.phase<-PI(1)) ms->phi2.phase += PI(2);
    return ms->phi2.phase;
}