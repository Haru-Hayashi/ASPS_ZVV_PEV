
#include "Inverter.h"

// *** prototype declaration (static)*** //

// *** static variable declaration *** //
/**
 * @brief convert table from VV number to switching array
 * 
 */
static uint8_t VoltageVectorPattern[8] = {
    0,  // V0  (000)
    1,  // V1  (001)
    3,  // V2  (011)
    2,  // V3  (010)
    6,  // V4  (110)
    4,  // V5  (100)
    5,  // V6  (101)
    7   // V7  (111)
};

/**
 * @brief a LUT for calculation switching count
 * 
 */
static uint8_t SwitchingCountTable[8] = {
    0,  // 0b000
    1,  // 0b001
    1,  // 0b010
    2,  // 0b011
    1,  // 0b100
    2,  // 0b101
    2,  // 0b110
    3   // 0b111
};

/**
 * @brief switching state (8 voltage vectors X 3 phases)
 * 
 */
// static bool SwitchingState[8][3] = {
//     {0, 0, 0},
//     {1, 0, 0},
//     {1, 1, 0},
//     {0, 1, 0},
//     {0, 1, 1},
//     {0, 0, 1},
//     {1, 0, 1},
//     {1, 1, 1}
// };

/**
 * @brief normalized voltage references
 * which can realize each voltage vector
 * 
 */
// static int8_t ModulationSwitchingState[8][3] = {
//     {-2, -2, -2},
//     { 2, -2, -2},
//     { 2,  2, -2},
//     {-2,  2, -2},
//     {-2,  2,  2},
//     {-2, -2,  2},
//     { 2, -2,  2},
//     { 2,  2,  2}
// };



// *** function definition *** //

/**
 * @brief Initialize the parameter of an inverter
 * 
 * @param ip initialized object
 * @param Vdc [V] DC link voltage
 */
bool Inv_UpdateOutputVoltage(Inv_InverterParam* ip, float Vdc){
    int i_vv; int j_axis;
 
    // インバータの出力可能な電圧ベクトルを計算
    ip->Vdc = Vdc;  // 直流リンク電圧
    for(i_vv=0;i_vv<8;++i_vv){
        // 各出力電圧ベクトルの初期化
        InitVect3(&(ip->vout[i_vv]));
        // SWPから3相電圧出力を計算（中性点電位考慮なし）
        for(j_axis=0;j_axis<3;++j_axis){
            if(SwitchingState[i_vv][j_axis]==0){
                ip->vout[i_vv].uvw[j_axis] = -ip->Vdc/2;
            }else{
                ip->vout[i_vv].uvw[j_axis] = ip->Vdc/2;
            }
        }
        // 3相電圧出力から2相電圧出力を計算
        uvw2ab(ip->vout[i_vv].uvw[0], ip->vout[i_vv].uvw[1], ip->vout[i_vv].uvw[2], &(ip->vout[i_vv].ab[0]), &(ip->vout[i_vv].ab[1]));
        // 2相電圧出力から3相電圧出力を再計算
        ab2uvw(ip->vout[i_vv].ab[0], ip->vout[i_vv].ab[1], &(ip->vout[i_vv].uvw[0]), &(ip->vout[i_vv].uvw[1]), &(ip->vout[i_vv].uvw[2]));
    }
    return 0;
}

/**
 * @brief Constructor of Inverter
 * 
 * @param ip Inverter structure
 * @param Vdc [V] DC link voltage
 * @return true 
 * @return false 
 */
bool Inv_Construct(Inv_InverterParam* ip, float Vdc){
    if(Inv_UpdateOutputVoltage(ip, Vdc)!=0) return -1;
    // 最初に設定する値
    ip->offset = 0;
    ip->out_swp = 0;
    ip->SWCnt = 0;
    ip->SWFreq = 0;
    return 0;
}

/**
 * @brief calculation swithing count from VV
 * 
 * @param ip invertor constructor
 * @param VoltageVector next vv
 * @return uint8_t 
 */
uint8_t Inv_CalcSwitchingCount(Inv_InverterParam* ip, uint8_t VoltageVector){
    return SwitchingCountTable[((VoltageVectorPattern[ip->out_swp])^(VoltageVectorPattern[VoltageVector]))];
}

/**
 * @brief calculation normalized voltage reference from vv
 * 
 * @param vv vv reference
 * @param phase uvw
 * @return uint8_t 
 */
int8_t Inv_CalcNVRFromVV(uint8_t vv, uint8_t phase){
    return ModulationSwitchingState[vv][phase];
}

/**
 * @brief Calculating the switching count and switching frequency
 * 
 * @param ip inverter object
 * @param VoltageVector voltage vector which will be selected.
 * @param ConstCount 0: direct VV determination | else: constant switching count (the total number of all phases) / control cycle
 * @return true 
 * @return false 
 */
// bool Inv_UpdateSwitchingCount(Inv_InverterParam* ip, uint8_t VoltageVector, uint8_t ConstCount){
//     unsigned int SWCnt_tmp = 0;
//     unsigned int MaxSwFreqCalcCycleCnt = (int)(CONTROL_FREQ/SWITCHING_FREQ_CALC_CYCLE);
//     static unsigned int SwFreqCalcCycleCnt = 0;
//     static unsigned int SWCnt_zfreq = 0;

//     if(ConstCount==0){
//         SWCnt_tmp = ip->SWCnt + Inv_CalcSwitchingCount(ip, VoltageVector);
//     }else{
//         SWCnt_tmp = ip->SWCnt + ConstCount;
//     }
//     ip->SWCnt = SWCnt_tmp;
//     SwFreqCalcCycleCnt += 1;
    
//     if(MaxSwFreqCalcCycleCnt == SwFreqCalcCycleCnt){
//         ip->SWFreq = (SWCnt_tmp - SWCnt_zfreq) * SWITCHING_FREQ_CALC_CYCLE;
//         SWCnt_zfreq = SWCnt_tmp;
//         SwFreqCalcCycleCnt = 0;
//     }
//     return 0;
// }