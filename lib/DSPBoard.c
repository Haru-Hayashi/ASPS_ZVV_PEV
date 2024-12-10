
#include "DSPBoard.h"

#include <stdbool.h>
#include <stdint.h>


// *** prototype declaration (static)*** //

// *** static variable declaration *** //
static bool IsFinishedOffsetAdjustment = 0;

// *** function definition *** //

/**
 * @brief Input the analog sensor values from a DSP board
 * 
 * @param bdn [in] board number
 * @param SensorData [out] input sensor value
 * @return true 
 * @return false 
 */
bool DSP_InputADData(int bdn, float* SensorData){
//    if(sizeof(SensorData)/sizeof(float)==8){
	    PEV_ad_start(bdn);
	    while (PEV_ad_in_st(bdn));
	    PEV_ad_in_grp(bdn, SensorData);

        return 0;
//    }else{
 //       return -1;
 //   }
}

/**
 * @brief Input the encoder's signal from a DSP board
 * 
 * @param bdn [in] board number
 * @param EncData[TIME_ORDER_NUM] [rad] mechanical angular
 * @param OmegaData[2] [rad/s] mechanical angular velocity [org, filtered]
 * @return true 
 * @return false 
 */
bool DSP_InputEncoderData(float f_sample, int bdn, float* EncData, float* OmegaData, LPF_tLowPassFilter* lpf){
	int i;
    float T_sample = 1.0/f_sample;

    //if(sizeof(EncData)/sizeof(float)==DSP_TIME_ORDER_NUM){
        for(i=DSP_TIME_ORDER_NUM-1;i>DSP_Z0;--i){
            EncData[i] = EncData[i-1];
        }
        // 機械角[rad]を格納 (0-2*M_PI)
        EncData[DSP_Z0] = (float)PEV_abz_read(bdn) / (float)ENC_MAX * 2 * M_PI;
        
        // 速度計算
    //    if(sizeof(OmegaData)/sizeof(float)==2){
            float delta_theta = EncData[DSP_Z0] - EncData[DSP_Z1];
        	float enc_tmp[2];
	        enc_tmp[0] = EncData[DSP_Z0];
        	enc_tmp[1] = EncData[DSP_Z1];
            if(fabs(delta_theta)>M_PI){
                if(delta_theta<0){
                    // 2*M_PI -> 0 のとき、前回角度を-2*M_PI
                    OmegaData[0] = (delta_theta + 2*M_PI) * f_sample;
                }else{
                    // 0 -> 2*M_PI のとき、前回角度を+2*M_PI
                    OmegaData[0] = (delta_theta - 2*M_PI) * f_sample;
                }
            }else{
                OmegaData[0] = delta_theta * f_sample;
            }
            // 速度LPF
            
            if(lpf->enable!=0){
                OmegaData[1] = LPF_CalcBDiff(lpf, LPF_Z0, OmegaData[0], 1000, T_sample);
            }else{
                OmegaData[1] = 0;
            }

            return 0;
    //    }
    //    return 1;
    //}else{
    //    return -1;
    //}
}
