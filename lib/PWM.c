
#include "PWM.h"

// *** variable declaration *** //
// int i;

// *** function definition *** //
// bool PWM_InitDeadtime(PWM_DEADTIME_CONPENSATION* dt, float Tc, float Td){
//     if(Tc-2*Td==0){
//         dt->Tc = Tc;
//         dt->Td = 0;
//     }else{
//         dt->Tc = Tc;
//         dt->Td = Td;
//     }
//     // Myway独自の変調率の補償用の係数
//     dt->compensate_coef[0] = Tc/(Tc-2*Td);
//     dt->compensate_coef[1] = (2*Td)/(Tc-2*Td);
//     return 0;
// }

/**
 * @brief デッドタイム補償
 * 
 * @param vref_cnp 電圧指令（処理後）
 * @param vref 電圧指令（被補償値）
 */
void PWM_Conpensate(PWM_DEADTIME_CONPENSATION* dt, float* vref){
    int i;

    for(i=0;i<3;++i){
        vref[i] = vref[i]*dt->compensate_coef[0] + dt->compensate_coef[1];
    }
    return;
}


/**
 * @brief 二相変調構造体の初期化
 * 
 * @param dpwm 二相変調器
 * @param min_val 貼り付け値
 */
void PWM_InitDPWM(PWM_DPWM* dpwm, float min_val){
    dpwm->min_val = min_val;
    return;
}


/**
 * @brief 下アーム方式二相変調の電圧指令計算
 * 
 * @param [in, out] vref 三相電圧指令\
 * @param [in] lower_voltage 下アーム電圧（-Vdc/2）
 */
// void PWM_CalcLowerArmDPWM(PWM_DPWM* dpwm, float* vref, float lower_voltage){
//     int i;

//     static float min_val;     // 最低となる相の電圧値
//     static uint32_t min_phase;   // 最低となる相

//     for(i=0;i<3;++i) dpwm->vref_org[i] = vref[i];   // 電圧指令コピー
//     // 貼り付ける相とその値を計算
//     min_val = MIN(vref[1], MIN(vref[2],vref[3]));
//     // 貼り付け処理
//     for(i=0;i<3;++i){
//         dpwm->vref_dpwm[i] = vref[i] - min_val + lower_voltage;
//     }
//     return;
// }




void PWM_InitSVM(PWM_SVM* svm, float Tc){
    int i;

    svm->Tc = Tc;
    for(i=0;i<2;++i) svm->vref[i] = 0;   // 電圧指令格納
    for(i=0;i<3;++i) svm->vv_ref[i] = 0;
    for(i=0;i<3;++i) svm->tvv_ref[i] = 0;

    for(i=0;i<3;++i) svm->Ton_cnt_ref[i] = 0;
    svm->area = 0;
    svm->sector = 0;

    return;
}

/**
 * @brief SVPWMの指令値計算
 * 
 * @param svm SVM
 * @param vref 電圧指令（αβ） [V]
 * @param Vdc 直流リンク電圧 [V]
 */
void PWM_CalcSVM(PWM_SVM* svm, float* vref, float Vdc, PWM_SVM_MODE mode){
    int i;

    // 出力
    static int vv_ref[3] = {0};
    static float tvv_ref[3] = {0};
    // 保持用変数
    static float vref_abs[2] = {0};
    static float zeta[2] = {0};
    static float zeta_abs[2] = {0};
    static float va_abs_sq3 = 0;
    static int area = 0;
    static int sector = 0;
    static float ton_phase_cnt_ref[3] = {0};

    // modeの代入
    svm->mode = mode;

    // ***** 瞬時空間ベクトル変調に基づき各相のオン時間を計算 ***** //
    vref_abs[0] = fabs(vref[0]);
    vref_abs[1] = fabs(vref[1]);
    va_abs_sq3 = vref_abs[0] * sqrt(3);
    // zetaの次元は無次元（カウント値）
    if(Vdc==0||vref[0]==0) zeta[0] = 0;
    else zeta[0] = sqrt(3.0/2.0) * vref[0]/Vdc*svm->Tc;
    if(Vdc==0||vref[1]==0) zeta[1] = 0;
    else zeta[1] = sqrt(1.0/2.0) * vref[1]/Vdc*svm->Tc;

    // vv_ref[0]は必ず零電圧ベクトル
    vv_ref[0] = 0;

    // *** 領域の判断と電圧ベクトル選択 *** //
    if((vref[0] < 0)&&(vref_abs[1] < va_abs_sq3)){
        area = 2;
        if(vref[1]>=0){
            sector = 3;
            vv_ref[1] = 3;
            vv_ref[2] = 4;
        }
        else{
            sector = 4;
            vv_ref[1] = 5;
            vv_ref[2] = 4;
        }
    }else{
        if(vref[1] >= 0){
            area = 1;
            if(vref_abs[1] > va_abs_sq3){
                sector = 2;
                vv_ref[1] = 3;
                vv_ref[2] = 2;
            }else{
                sector = 1;
                vv_ref[1] = 1;
                vv_ref[2] = 2;
            }
        }else{
            area = 3;
            if(vref_abs[1] > va_abs_sq3){
                sector = 5;
                vv_ref[1] = 5;
                vv_ref[2] = 6;
            }
            else{
                sector = 6;
                vv_ref[1] = 1;
                vv_ref[2] = 6;
            }
        }
    }

    // *** オン時間の計算と持続時間への変換 *** //
    // 格納順：V0→奇数VV→偶数VV→V7→偶数VV→奇数VV→V0
    // t_V0 = (ゲート出力周期 - 出力時間が一番長い相の時間)/2　（V0とV7で分割）
    // t_Vodd = 出力時間が一番長い相の時間（1相オン） - 出力が短い相の時間（2相オン）
    // t_Veven = 出力が短い相の時間（2相オン）
    switch(area){
        case 1:
            // 各相オン時間
            zeta_abs[1] = fabs(zeta[1]);
            ton_phase_cnt_ref[0] = zeta[0] + zeta_abs[1];
            ton_phase_cnt_ref[1] = 2 * zeta_abs[1];
            ton_phase_cnt_ref[2] = 0;
            // 持続時間への変換
            if(ton_phase_cnt_ref[0]>=ton_phase_cnt_ref[1]){
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[0]);
                tvv_ref[1] = ton_phase_cnt_ref[0] - ton_phase_cnt_ref[1];
                tvv_ref[2] = ton_phase_cnt_ref[1];
            }else{
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[1]);
                tvv_ref[1] = ton_phase_cnt_ref[1]-ton_phase_cnt_ref[0];
                tvv_ref[2] = ton_phase_cnt_ref[0];
            }
            break;
        case 2:
            // 各相オン時間
            zeta_abs[0] = fabs(zeta[0]);
            ton_phase_cnt_ref[0] = 0;
            ton_phase_cnt_ref[1] = zeta_abs[0] + zeta[1];
            ton_phase_cnt_ref[2] = zeta_abs[0] - zeta[1];
            // 持続時間への変換
            if(ton_phase_cnt_ref[1]>=ton_phase_cnt_ref[2]){
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[1]);
                tvv_ref[1] = ton_phase_cnt_ref[1]-ton_phase_cnt_ref[2];
                tvv_ref[2] = ton_phase_cnt_ref[2];
            }else{
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[2]);
                tvv_ref[1] = ton_phase_cnt_ref[2]-ton_phase_cnt_ref[1];
                tvv_ref[2] = ton_phase_cnt_ref[1];
            }
            break;
        case 3:
            // 各相オン時間
            zeta_abs[1] = fabs(zeta[1]);
            ton_phase_cnt_ref[0] = zeta[0] + zeta_abs[1];
            ton_phase_cnt_ref[1] = 0;
            ton_phase_cnt_ref[2] = 2 * zeta_abs[1];
            // 持続時間への変換
            if(ton_phase_cnt_ref[2]>=ton_phase_cnt_ref[0]){
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[2]);
                tvv_ref[1] = ton_phase_cnt_ref[2]-ton_phase_cnt_ref[0];
                tvv_ref[2] = ton_phase_cnt_ref[0];
            }else{
                tvv_ref[0] = (svm->Tc - ton_phase_cnt_ref[0]);
                tvv_ref[1] = ton_phase_cnt_ref[0]-ton_phase_cnt_ref[2];
                tvv_ref[2] = ton_phase_cnt_ref[2];
            }
            break;
        default:
            ton_phase_cnt_ref[0] = 0;
            ton_phase_cnt_ref[1] = 0;
            ton_phase_cnt_ref[2] = 0;
            tvv_ref[0] = svm->Tc;
            tvv_ref[1] = svm->Tc;
            tvv_ref[2] = svm->Tc;
            break;
    }

    for(i=0;i<3;++i) svm->vv_ref[i] = vv_ref[i];
    for(i=0;i<3;++i) svm->tvv_ref[i] = tvv_ref[i];
    for(i=0;i<3;++i) svm->Ton_cnt_ref[i] = ton_phase_cnt_ref[i];
    svm->area = area;
    svm->sector = sector;
    return;
}