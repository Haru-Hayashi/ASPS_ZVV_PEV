#ifndef PWM_H
#define PWM_H

// *** header file include *** //
#include "Common.h"
#include "../UserDefined.h"
#include "Vector.h"
// *** macro definition *** //

// *** variable definition *** //
typedef enum{
  SVM_3PHASE = 0,
  SVM_LOWER,
  SVM_UPPER,
  SVM_UPLOW,
  SVM_MODE_NUM
} PWM_SVM_MODE;

// *** type definition *** //
// デッドタイム補償
typedef struct{
  float Tc;                 // 制御周期 [s]
  float Td;                 // デッドタイム [s]
  float compensate_coef[2]; // Myway独自の変調率の補償用の係数
} PWM_DEADTIME_CONPENSATION;

// 二相変調：Discontinuous PWM
typedef struct{
  float min_val;  // 貼り付け値
  float vref_org[3];  // 電圧指令（処理前）
  float vref_dpwm[3]; // 電圧指令（処理後）
} PWM_DPWM;


// 空間ベクトル変調：Space Vector PWM
typedef struct{
  float Tc;               // 制御周期 [s]
  float vref[2];          // 入力：αβ電圧指令 [V]
  int vv_ref[3];  // 出力：非零電圧ベクトル2つ
  float tvv_ref[3];       // 出力：零VVおよび非零VV2つの出力時間
  PWM_SVM_MODE mode;      // 変調方式
  // *** デバッグ用 *** //
  float Ton_cnt_ref[3];   // 各相オン時間
  int area;               // 領域（奇数VV/偶数VVの組）
  int sector;             // セクタ（六角形）
} PWM_SVM;


// *** prototype definition *** //
//bool PWM_InitDeadtime(PWM_DEADTIME_CONPENSATION* dt, float Tc, float Td);
void PWM_CalcConpensation(PWM_DEADTIME_CONPENSATION* dt, float* vref);

void PWM_InitDPWM(PWM_DPWM* dpwm, float min_val);
// void PWM_CalcLowerArmDPWM(PWM_DPWM* dpwm, float* vref); // こっちに変える
void PWM_CalcLowerArmDPWM(PWM_DPWM* dpwm, float* vref, float lower_voltage);

void PWM_InitSVM(PWM_SVM* svm, float Tc);
void PWM_CalcSVM(PWM_SVM* svm, float* vref, float Vdc, PWM_SVM_MODE mode);

// *** prototype definition (static)*** //

#endif