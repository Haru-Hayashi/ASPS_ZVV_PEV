#ifndef USERDEFINED_H_
#define USERDEFINED_H_

#include <mwio4.h>
#include <math.h>

#define far

// * 制御設定 * //

// #define CONTROL_FREQ 2000
// #define SAMPLING_FREQ 40000
#define CTRL_BDN 1      //BoarD Number for control default→0
#define SENS_BDN 0      //BoarD Number for sensor  default→1
#define DAC_BDN 3
#define FPGA_BDN 4

#define REF_UPDATE_FREQ CONTROL_FREQ    // 電圧指令値更新周波数は合わせる
#define DEADTIME 200
#define ENC_MAX 8191    //2048*4=2^13
#define M_PI 3.14159
// * センサ設定 * //
#define CURRENT_SENSOR_RANGE 50.0/3.0//12.5
#define DCLINK_VOLTAGE_RANGE 1016
#define TORQUE_SENSOR_RANGE 20.0
#define DCLINK_CURRENT_RANGE 31.25
#define SWITCHING_FREQ_CALC_CYCLE 10000 //[Hz]
// * 帯域設定 * //
#define BW_OMEGARE 800  //速度応答ローパス

// * 零近似（計算結果において完全な0は存在しない） * //
#define I1_EPSILON (1e-3) //-2
#define EPSILON (1e-3)

// ASPSのテーブル書き込み分割数 
#define WRITE_DIVISION 4


#endif
