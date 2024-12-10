#ifndef MWIO4_H_
#define MWIO4_H_

#define interrupt
#define CSL_IntcVectId int

typedef enum{
    FALSE = 0,
    TRUE
} FLG;

extern float* expert_input_ad;
extern float expert_input_enc;

extern float gVout_Nrm[3];

// 割り込み
void int1_ack(void);
void int2_ack(void);

// AD変換
void PEV_ad_start(int bdn);
int PEV_ad_in_st(int bdn);
void PEV_ad_in_grp(int bdn, float* ad_data);

// DIO
void PEV_pio_set_bit(int bdn, int pin);
void PEV_pio_clr_bit(int bdn, int pin);

// エンコーダ
float PEV_abz_read(int bdn);

// タイマ
void int_enable(void);               //全割り込みを有効化
void C6657_timer0_init(unsigned int prd);     // 1e6[us]秒のタイマ割り込み設定
void C6657_timer0_init_vector(void (*func)(int), unsigned int id);    //割り込み関数の設定
void C6657_timer0_enable_int(void);  //割り込み許可
void C6657_timer0_start(void);   // スタート
unsigned int C6657_timer0_read(void);
void C6657_timer0_clear(void);
void C6657_timer0_clear_eventflag(void);
void C6657_timer0_stop(void);

// 高速算術演算
float mwsin(float theta);
float mwcos(float theta);
float mwabs(float value);
float mwsqrt(float value);
float mwarctan2(float y, float x);

// 座標変換
void uvw2ab(float u, float v, float w, float* a, float*b);
void ab2dq(float a, float b, float theta, float* d, float* q);
void dq2ab(float d, float q, float theta, float* a, float*b);
void ab2uvw( float a, float b, float* u, float* v, float* w);

// インバータ
PEV_inverter_set_uvw(int bdn, float u, float v, float w, int num);

#endif