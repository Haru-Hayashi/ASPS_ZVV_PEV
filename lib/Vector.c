
#include "Vector.h"
#include "../UserDefined.h"


/**
 * @brief Initialize VCT_tVect2
 * 
 * @param vs2 pointer of VCT_tVect2
 */
void VCT2_InitFunc(VCT_tVect2State* vs2){
    // 演算時間短縮のため展開
    vs2->ab[0] = 0;
    vs2->ab[1] = 0;

    vs2->dq[0] = 0;
    vs2->dq[1] = 0;

    vs2->phase = 0;
    vs2->nrm = 0;
}

/**
 * @brief Initialize VCT_tVect3State
 * 
 * @param vs3 pointer of VCT_tVect3State
 */

/**
 * @brief Initialize VCT_tVect2State
 * 
 * @param vs2 pointer of VCT_tVect2State
 */
void InitVect2(VCT_tVect2State* vs2){
    int i;
    for(i=0;i<2;++i){
        vs2->ab[i] = 0;
        vs2->dq[i] = 0;
    }
    vs2->phase = 0;
    vs2->nrm = 0;
}

/**
 * @brief Initialize VCT_tVect3State
 * 
 * @param vs3 pointer of VCT_tVect3State
 */
void InitVect3(VCT_tVect3State* vs3){
    int i;
    for(i=0;i<2;++i){
        vs3->ab[i] = 0;
        vs3->dq[i] = 0;
        vs3->uvw[i] = 0;
    }
    vs3->uvw[2] = 0;
    vs3->phase = 0;
    vs3->nrm[1] = 0;
}


/**
 * @brief 座標変換
 * 
 * @param vect3 vector object
 * @param org the axis before convertion
 * @param cnv the axis after convertion
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(VCT3_ConvAxis, ".CODE_ON_HIGHER_SPEED")
#endif
void VCT3_ConvAxis(VCT_tVect3State* vect3, VCT_eAXIS org, VCT_eAXIS cnv){
    switch(org){
        case VCT_AXIS_UVW:
            switch (cnv){
                case VCT_AXIS_AB:
                    uvw2ab(vect3->uvw[0], vect3->uvw[1], vect3->uvw[2], &(vect3->ab[0]), &(vect3->ab[1]));
                    break;
                case VCT_AXIS_DQ:
                    uvw2ab(vect3->uvw[0], vect3->uvw[1], vect3->uvw[2], &(vect3->ab[0]), &(vect3->ab[1]));
                    ab2dq(vect3->ab[0], vect3->ab[1], vect3->phase, &(vect3->dq[0]), &(vect3->dq[1]));
                default:
                    break;
            }
            break;
        case VCT_AXIS_AB:
            switch (cnv){
                case VCT_AXIS_UVW:
                    ab2uvw(vect3->ab[0], vect3->ab[1], &(vect3->uvw[0]), &(vect3->uvw[1]), &(vect3->uvw[2]));
                    break;
                case VCT_AXIS_DQ:
                    ab2dq(vect3->ab[0], vect3->ab[1], vect3->phase, &(vect3->dq[0]), &(vect3->dq[1]));
                default:
                    break;
            }
            break;
        case VCT_AXIS_DQ:
            switch (cnv){
                case VCT_AXIS_UVW:
                    dq2ab(vect3->dq[0], vect3->dq[1], vect3->phase, &(vect3->ab[0]), &(vect3->ab[1]));
                    ab2uvw(vect3->ab[0], vect3->ab[1], &(vect3->uvw[0]), &(vect3->uvw[1]), &(vect3->uvw[2]));
                    break;
                case VCT_AXIS_AB:
                    dq2ab(vect3->dq[0], vect3->dq[1], vect3->phase, &(vect3->ab[0]), &(vect3->ab[1]));
                default:
                    break;
            }
            break;
    }
    return;
}

/**
 * @brief calculation voltage vector's norm
 * 
 * @param vect3 vector object
 * @param org base axis
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(VCT3_CalcPhasor, ".CODE_ON_HIGHER_SPEED")
#endif
void VCT3_CalcPhasor(VCT_tVect3State* vect3, VCT_eAXIS org){
    float x;
    float y;

    switch(org){
        case VCT_AXIS_UVW:
            uvw2ab(vect3->uvw[0], vect3->uvw[1], vect3->uvw[2], &(x), &(y));
            break;
        case VCT_AXIS_AB:
            x = vect3->ab[0];
            y = vect3->ab[1];
            break;
        case VCT_AXIS_DQ:
            x = vect3->dq[0];
            y = vect3->dq[1];
            break;
        default:
            x = 0; y = 0;
    }
    vect3->nrm[1] = (float)sqrt(x*x+y*y);
    vect3->nrm[0] = (float)SQ23*vect3->nrm[1];
    vect3->phase = (float)atan2f(y,x);
    return;
}


#ifndef MATLAB_SIM
#pragma CODE_SECTION(VCT3_SetZeroPhase, ".CODE_ON_HIGHER_SPEED")
#endif
/**
 * @brief 積分誤差の排除
 * 三相電圧和が0であり三相電圧積分和も0となるため、これを0となるように調節する
 * 
 * @param vect 三相電圧積分値
 * @param zero_val 合わせたい零相電圧値
 */
void VCT3_SetZeroPhase(VCT_tVect3State* vect, float zero_val){
    float common_mode;
    common_mode = (vect->uvw[0] + vect->uvw[1] + vect->uvw[2])/3.0 + zero_val;
    vect->uvw[0] = vect->uvw[0] - common_mode;
    vect->uvw[1] = vect->uvw[1] - common_mode;
    vect->uvw[2] = vect->uvw[2] - common_mode;
    return;
}

#ifndef MATLAB_SIM
#pragma CODE_SECTION(VCT3_CalcVectorError, ".CODE_ON_HIGHER_SPEED")
#endif
/**
 * @brief ベクトルの減算
 * 
 * @param [out] vect_err 偏差
 * @param [in] vect_org 被減数
 * @param [in] vect_sub 減数
 * @param [in] axis 座標系
 */
void VCT3_CalcVectorError(VCT_tVect3State* vect_err, VCT_tVect3State vect_org, VCT_tVect3State vect_sub, VCT_eAXIS axis){
    switch(axis){
        case VCT_AXIS_UVW:
            vect_err->uvw[0] = vect_org.uvw[0] - vect_sub.uvw[0];
            vect_err->uvw[1] = vect_org.uvw[1] - vect_sub.uvw[1];
            vect_err->uvw[2] = vect_org.uvw[2] - vect_sub.uvw[2];
            break;
        case VCT_AXIS_AB:
            vect_err->ab[0] = vect_org.ab[0] - vect_sub.ab[0];
            vect_err->ab[1] = vect_org.ab[1] - vect_sub.ab[1];
            break;
        default:
            break;
    }
    return;
}


/**
 * @brief calculation the sector from phase
 * 
 * @param vect3 vector object
 * @return sector:1-6:[0-60-120-180-...]
 */
#ifndef MATLAB_SIM
#pragma CODE_SECTION(VCT3_CalcSector, ".CODE_ON_HIGHER_SPEED")
#endif
uint8_t VCT3_CalcSector(VCT_tVect3State* vect3, float phase_offset){
    /*
    static float phase_tmp;
    phase_tmp = vect3->phase + phase_offset;
    while(phase_tmp<0) phase_tmp += 2*M_PI;
    while(phase_tmp>2*M_PI) phase_tmp -= 2*M_PI;
    return (int)(phase_tmp * 180.0/M_PI /60.0) + 1;
    */
    float a, b, absa, absb, sq3a;
    float c, s;
    c = cos(phase_offset);
    s = sin(phase_offset);
    a = vect3->ab[0] *  c + vect3->ab[1] * s;
    b = vect3->ab[0] * -s + vect3->ab[1] * c;
    absa = abs(a);
    absb = abs(b);
    sq3a = sqrt(3)*absa;

    if(b>0){
        if(sq3a<absb) return 2;
        else if(a>0) return 1;
        else return 3;
    }else{
        if(sq3a<absb) return 5;
        else if(a>0) return 6;
        else return 4;
    }
}

/**
 * @brief 角速度の演算
 * 
 * @param [in] vect3 ベクトル（今回の位相が格納されているものとする）
 * @param [in] zphase 位相の前回値
 * @param [out] omega 角速度
 */
// void VCT3_CalcOmega(VCT_tVect3State* vect3, float zphase, float* omega){
//     float phs_err = vect3->phase - zphase;

//     // -pi <-> +pi のときは符号が変わり、かつ2pi近くの差がある
//     if((SGN(vect3->phase)!=SGN(zphase)) && (SGN(phs_err)*phs_err>PI(1))){
//         // CWのとき
//         if(phs_err<0)   *omega = (phs_err + PI(2)) * CONTROL_FREQ;
//         // CCWのとき
//         else            *omega = (phs_err - PI(2)) * CONTROL_FREQ;
//     }
//     // 通常時
//     else *omega = phs_err * CONTROL_FREQ;

//     return;
// }