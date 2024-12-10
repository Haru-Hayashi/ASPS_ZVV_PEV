#ifndef DSPBOARD_H_
#define DSPBOARD_H_

// *** header file include *** //
#include <math.h>

#include "LowPassFilter.h"
#include "../UserDefined.h"

// *** macro definition *** //

// *** type definition *** //
typedef enum{
    DSP_Z0 = 0,
    DSP_Z1,
    DSP_TIME_ORDER_NUM
} DSP_TIME_ORDER;


// *** prototype definition *** //
bool DSP_InputADData(int bdn, float* SensorData);
bool DSP_InputEncoderData(float f_sample, int bdn, float* EncData, float* OmegaData, LPF_tLowPassFilter* lpf);

// *** prototype definition (static)*** //

#endif
