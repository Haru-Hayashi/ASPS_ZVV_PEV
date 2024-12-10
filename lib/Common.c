
#include "Common.h"

// *** prototype declaration (static)*** //

// *** variables *** //
static int i;

// *** function definition *** //

/**
 * @brief 最大値と要素番号を抽出
 * 
 * @param [in] num 対称とする配列
 * @param [in] numlen 要素数
 * @param [out] max 最大値
 * @param [out] index 要素番号
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(COM_Max, ".CODE_ON_HIGHER_SPEED")
#endif
int COM_Max(float* num, uint32_t numlen){
    static float maximum;
    static int index;
    // 初期化
    maximum = num[0];
    index = 0;
    // 比較
    for(i=1;i<numlen;++i){
        maximum = MAX(num[i],maximum);
        index = (num[i]==maximum)?(i):(index);
    }
    return index;
}


/**
 * @brief 最大値と要素番号を抽出(2つver)。MAX2_INDEXと等価。
 * 
 * @param [in] num 対称とする配列
 * @param [in] numlen 要素数
 * @param [out] int 要素番号
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(Com2_Max, ".CODE_ON_HIGHER_SPEED")
#endif
int COM_Max2(float num1, float num2){
    if(num1>num2) return 0;
    else return 1;
}

/**
 * @brief 最小値と要素番号を抽出
 * 
 * @param [in] num 対称とする配列
 * @param [in] numlen 要素数
 * @param [out] min 最小値
 * @param [out] index 要素番号
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(COM_Min, ".CODE_ON_HIGHER_SPEED")
#endif
int COM_Min(float* num, uint32_t numlen){
    static float minimum;
    static int index;
    // 初期化
    minimum = num[0];
    index = 0;
    // 比較
    for(i=1;i<numlen;++i){
        if(num[i]<minimum){
            minimum = num[i];
            index = i;
        }
    }
    return index;
}


/**
 * @brief 最小値と要素番号を抽出(2つver)
 * 
 * @param [in] num 対称とする配列
 * @param [in] numlen 要素数
 * @param [out] min 最小値
 * @param [out] index 要素番号
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(Com2_Min, ".CODE_ON_HIGHER_SPEED")
#endif
int Com2_Min(float num1, float num2){
    if(num1<num2) return 0;
    else return 1;
}

/**
 * @brief 最小値と要素番号を抽出(3つver)
 * 
 * @param [in] num 対称とする配列
 * @param [in] numlen 要素数
 * @param [out] min 最小値
 * @param [out] index 要素番号
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(Com3_Min, ".CODE_ON_HIGHER_SPEED")
#endif
int Com3_Min(float num1, float num2, float num3){
    if(num1<num2){
        if(num1<num3) return 0;
        else return 2;
    }
    // if(num1>num2)
    else{
        if(num2<num3) return 1;
        else return 2;
    }
}

void COM_MinIndex(float* value, int num, COM_tIndex* ret){
    // int size = (int)(sizeof(value) / sizeof(float)); // 無理っぽい
    ret->value = value[0];
    ret->index = 0;
    for(i=1;i<num;++i){
        if(ret->value > value[i]){
            ret->value = value[i];
            ret->index = i;
        }
    }
}

/**
 * @brief リミッタ（誤差出力付き）
 * 
 * @param sig 信号（被リミット値）
 * @param lim_val リミット値
 * @param delta 誤差（sig-limit）
 * @return float リミットされた値
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(COM_Max, ".CODE_ON_HIGHER_SPEED")
#endif
float COM_Limiter(float sig, float lim_val, float* delta){
    if(SGN(sig)*sig>=lim_val){
        *delta = SGN(sig) * (sig - SGN(sig)*lim_val);
        return SGN(sig)*lim_val;
    }else{
        *delta = 0;
        return sig;
    }
}
