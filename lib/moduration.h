#ifndef MODURATION_H
#define MODURATION_H


typedef struct{
int VV_ref[8];
float VV_ref_time[8];
int sector;
float Vtheta;
float tu;
float tv;
float tw;
} SVM_Calc;


void SVM_Calculation(SVM_Calc *svm, float V_a, float V_b, float Vdc, float Ts);
void SVM_Calculation_Init(SVM_Calc *svm);

#endif