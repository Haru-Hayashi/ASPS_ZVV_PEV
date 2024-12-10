#include "mwio4.h"

#include <stdbool.h>

float gVout_Nrm[3] = {0};

void int1_ack(void){
    return;
}
void int2_ack(void){
    return;
}

void PEV_ad_start(int bdn){
    return;
}
int PEV_ad_in_st(int bdn){
    return 0;
}
void PEV_ad_in_grp(int bdn, float* ad_data){
    ad_data[0] = expert_input_ad[0];
    ad_data[1] = expert_input_ad[1];
    ad_data[2] = expert_input_ad[2];
    ad_data[3] = expert_input_ad[3];
    ad_data[4] = expert_input_ad[4];
    ad_data[5] = expert_input_ad[5];
    ad_data[6] = expert_input_ad[6];
    ad_data[7] = expert_input_ad[7];
}

void PEV_pio_set_bit(int bdn, int pin){
    return;
}

void PEV_pio_clr_bit(int bdn, int pin){
    return;
}



float PEV_abz_read(int bdn){
    return expert_input_enc * (float)ENC_MAX / (2*M_PI);
}


void int_enable(){
    return;
}
void C6657_timer0_init(unsigned int prd){
    return;
}
void C6657_timer0_init_vector(void (*func)(int), unsigned int id){
    return;
}
void C6657_timer0_enable_int(void){
    return;
}
void C6657_timer0_start(void){
    return;
}

unsigned int C6657_timer0_read(void){
    return 0;
}

void C6657_timer0_clear(void){
    return;
}

void C6657_timer0_clear_eventflag(void){
    return;
}
void C6657_timer0_stop(void){
    return;
}

float mwsin(float theta){
    return sin(theta);
}

float mwcos(float theta){
    return cos(theta);
}

float mwabs(float value){
    return fabs(value);
}

float mwsqrt(float value){
    return sqrt(value);
}

float mwarctan2(float y, float x){
    return atan2(y, x);
}

void uvw2ab(float u, float v, float w, float* a, float*b){
    *a = (float)(sqrt(2.0/3.0)*(u - 0.5*v - 0.5*w));
    *b = (float)(sqrt(2.0/3.0)*sqrt(3)/2.0*(v-w));
}

void ab2dq(float a, float b, float theta, float* d, float* q){
    float c = (float)cos(theta);
    float s = (float)sin(theta);
    *d = c*a + s*b;
    *q = -s*a + c*b;
}

void dq2ab(float d, float q, float theta, float* a, float*b){
    float c = (float)cos(theta);
    float s = (float)sin(theta);
    *a = c*d - s*q;
    *b = s*d + c*q;
}

void ab2uvw( float a, float b, float* u, float* v, float* w){
    *u = (float)(sqrt(2.0/3.0)*a);
    *v = (float)(sqrt(2.0/3.0)*(-0.5*a + sqrt(3)/2.0*b));
    *w = (float)(sqrt(2.0/3.0)*(-0.5*a - sqrt(3)/2.0*b));
}



PEV_inverter_set_uvw(int bdn, float u, float v, float w, int num){
    gVout_Nrm[0] = (bool)(u>0);
    gVout_Nrm[1] = (bool)(v>0);
    gVout_Nrm[2] = (bool)(w>0);
}
