#ifndef VECTOR_H_
#define VECTOR_H_

// *** header file include *** //
#include "Common.h"
// *** macro definition *** //

// *** type definition *** //
/**
 * @enum VCT_AXIS
 * @brief 座標系
 * 
 */
typedef enum{
    VCT_AXIS_UVW = 0,
    VCT_AXIS_AB,
    VCT_AXIS_DQ,
    VCT_AXIS_NUM
} VCT_eAXIS;

/**
 * @enum VCT_AXIS_MAX
 * @brief 各座標系の最大値
 * 
 */
typedef enum{
    VCT_AXIS_MAX_UVW = 3,
    VCT_AXIS_MAX_AB = 2,
    VCT_AXIS_MAX_DQ = 2,
    VCT_AXIS_MAX_NUM = 3
} VCT_eAXIS_SIZE;


/**
 * @struct VCT_tVect3State
 * @brief 三相交流用の変数
 */
typedef struct{
    float uvw[VCT_AXIS_MAX_UVW];
    float ab[VCT_AXIS_MAX_AB];
    float dq[VCT_AXIS_MAX_DQ];
    float nrm[2];   // [3相座標での大きさ, 2相座標での大きさ]
    float phase;
} VCT_tVect3State;

/**
 * @struct VCT_tVect3State
 * @brief 二相交流用の変数
 */
typedef struct{
    float ab[VCT_AXIS_MAX_AB];
    float dq[VCT_AXIS_MAX_DQ];
    float nrm;
    float phase;
} VCT_tVect2State;


// *** prototype definition *** //
void VCT2_InitFunc(VCT_tVect2State* vs2);
void VCT3_InitFunc(VCT_tVect3State* vs3);
void VCT3_ConvAxis(VCT_tVect3State* vect3, VCT_eAXIS org, VCT_eAXIS cnv);
void VCT3_CalcPhasor(VCT_tVect3State* vect3, VCT_eAXIS org);
uint8_t VCT3_CalcSector(VCT_tVect3State* vect3, float phase_offset);

void VCT3_SetZeroPhase(VCT_tVect3State* vect, float zero_val);
void VCT3_CalcVectorError(VCT_tVect3State* vect_err, VCT_tVect3State vect_org, VCT_tVect3State vect_sub, VCT_eAXIS axis);


void VCT3_CalcOmega(VCT_tVect3State* vect3, float zphase, float* omega);
#endif
