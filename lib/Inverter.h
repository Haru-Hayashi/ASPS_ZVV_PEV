#ifndef INVERTER_H_
#define INVERTER_H_

// *** header file include *** //
#include "Vector.h"
#include "../UserDefined.h"
#include <stdint.h>
#include <stdbool.h>

// *** macro definition *** //

// *** type definition *** //

/**
 * @brief Inverter structure
 * 
 * @param Vdc [V]       DC link voltage
 * @param offset [V]    DC link voltage sensor offset
 * @param out_swp       current output switching pattern
 * @param vout[8] [V]   voltage output of each voltage vector
 * 
 */
typedef struct{
    float Vdc;
    float offset;
    unsigned int SWCnt;
    float SWFreq;
    uint8_t out_swp;
    VCT_tVect3State vout[8];
} Inv_InverterParam;

static bool SwitchingState[8][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {1, 1, 0},
    {0, 1, 0},
    {0, 1, 1},
    {0, 0, 1},
    {1, 0, 1},
    {1, 1, 1}
};

static int8_t ModulationSwitchingState[8][3] = {
    {-2, -2, -2},
    { 2, -2, -2},
    { 2,  2, -2},
    {-2,  2, -2},
    {-2,  2,  2},
    {-2, -2,  2},
    { 2, -2,  2},
    { 2,  2,  2}
};


// *** prototype definition *** //
bool Inv_UpdateOutputVoltage(Inv_InverterParam* ip, float Vdc);
bool Inv_Construct(Inv_InverterParam* ip, float Vdc);
uint8_t Inv_CalcSwitchingCount(Inv_InverterParam* ip, uint8_t VoltageVector);
int8_t Inv_CalcNVRFromVV(uint8_t vv, uint8_t phase);
bool Inv_UpdateSwitchingCount(Inv_InverterParam* ip, uint8_t VoltageVector, uint8_t ConstCount);
// *** prototype definition (static)*** //

#endif
