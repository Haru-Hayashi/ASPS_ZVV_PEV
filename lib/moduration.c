#include "moduration.h"
#include "math.h"

#define PI 3.14159265358979
#define PI2_3 2.094395102
#define SQRT2 1.414213562


void SVM_Calculation_Init(SVM_Calc *svm){
    int i;
    for(i = 0; i < 8; ++i){
        svm->VV_ref[i] = 0;
        svm->VV_ref_time[i] = 0;
    }

    svm->Vtheta = 0.0;
    svm->sector = 0;
    svm->tu = 0.0;
    svm->tv = 0.0;
    svm->tw = 0.0;
}

void SVM_Calculation(SVM_Calc *svm, float V_a, float V_b,float Vdc, float Ts){
    int sector = 0;
    int Fu = 0, Fv = 0, Fw = 0;
    float Vtheta = 0;
    //float Vdc = 282.0;
    float tu = 0.0, tv = 0.0, tw = 0.0;
    float Vt[8] = {0};
    int Su = 0, Sv = 0, Sw = 0;
    float t_out[8] = {0};
    // int VV_ref[8] = {0};
    // float VV_ref_time[8] = {0};

     Vtheta = atan2(V_b, V_a);
    // svm->Vtheta = atan2(V_b, V_a);
    svm->Vtheta = Vtheta;

    if((Vtheta >= 0) && (Vtheta < PI2_3)){
        sector = 1;
        Fu = 0;
        Fv = 0; 
        Fw = 1; 
    }else 
    if((Vtheta >= PI2_3) && (Vtheta <= PI)){
        sector = 2;
        Fu = 1;
        Fv = 0; 
        Fw = 0; 
    }else
    if((Vtheta < -PI2_3) && (Vtheta >= -PI)){
        sector = 2;
        Fu = 1;
        Fv = 0; 
        Fw = 0; 
    }else
    if((Vtheta < 0) && (Vtheta >= -PI2_3)){
        sector = 3;
        Fu = 0;
        Fv = 1; 
        Fw = 0; 
    }

    tu = Ts/sqrt(2)/Vdc*((Fv+Fw)*sqrt(3)*V_a+(-Fv+Fw)*V_b);
    tv = Ts/sqrt(2)/Vdc*((-Fu)*sqrt(3)*V_a+(2*Fw+Fu)*V_b);
    tw = Ts/sqrt(2)/Vdc*((-Fu)*sqrt(3)*V_a+(-Fu-2*Fv)*V_b);

    if(sector == 1){

        if(tu > tv){
            svm->VV_ref[0] = 2;
            svm->VV_ref[1] = 1;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 1;
            svm->VV_ref[4] = 2;
            svm->VV_ref[5] = 2;
            svm->VV_ref[6] = 2;

            Vt[0] = tv*0.5;
            Vt[1] = (tu - tv)*0.5;
            Vt[2] = Ts-tv-(tu-tv);
            Vt[3] = (tu - tv)*0.5;
            Vt[4] = tv*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
        if(tu < tv){
            svm->VV_ref[0] = 2;
            svm->VV_ref[1] = 3;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 3;
            svm->VV_ref[4] = 2;
            svm->VV_ref[5] = 2;
            svm->VV_ref[6] = 2;

            Vt[0] = tu*0.5;
            Vt[1] = (tv - tu)*0.5;
            Vt[2] = Ts-tu-(tv-tu);
            Vt[3] = (tv - tu)*0.5;
            Vt[4] = tu*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
    
    }
    if(sector == 2){

        if(tv > tw){
            svm->VV_ref[0] = 4;
            svm->VV_ref[1] = 3;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 3;
            svm->VV_ref[4] = 4;
            svm->VV_ref[5] = 4;
            svm->VV_ref[6] = 4;

            Vt[0] = tw*0.5;
            Vt[1] = (tv - tw)*0.5;
            Vt[2] = Ts-tw-(tv-tw);
            Vt[3] = (tv - tw)*0.5;
            Vt[4] = tw*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
        if(tv < tw){
            svm->VV_ref[0] = 4;
            svm->VV_ref[1] = 5;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 5;
            svm->VV_ref[4] = 4;
            svm->VV_ref[5] = 4;
            svm->VV_ref[6] = 4;

            Vt[0] = tv*0.5;
            Vt[1] = (tw - tv)*0.5;
            Vt[2] = Ts-tv-(tw-tv);
            Vt[3] = (tw - tv)*0.5;
            Vt[4] = tv*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
    
    }
    if(sector == 3){

        if(tw > tu){
            svm->VV_ref[0] = 6;
            svm->VV_ref[1] = 5;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 5;
            svm->VV_ref[4] = 6;
            svm->VV_ref[5] = 6;
            svm->VV_ref[6] = 6;

            Vt[0] = tu*0.5;
            Vt[1] = (tw - tu)*0.5;
            Vt[2] = Ts-tu-(tw-tu);
            Vt[3] = (tw - tu)*0.5;
            Vt[4] = tu*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
        if(tw < tu){
            svm->VV_ref[0] = 6;
            svm->VV_ref[1] = 1;
            svm->VV_ref[2] = 0;
            svm->VV_ref[3] = 1;
            svm->VV_ref[4] = 6;
            svm->VV_ref[5] = 6;
            svm->VV_ref[6] = 6;

            Vt[0] = tw*0.5;
            Vt[1] = (tu - tw)*0.5;
            Vt[2] = Ts-tw-(tu-tw);
            Vt[3] = (tu - tw)*0.5;
            Vt[4] = tw*0.5;
        
            svm->VV_ref_time[0] = Vt[0];
            svm->VV_ref_time[1] = svm->VV_ref_time[0]+Vt[1];
            svm->VV_ref_time[2] = svm->VV_ref_time[1]+Vt[2];
            svm->VV_ref_time[3] = svm->VV_ref_time[2]+Vt[3];
            svm->VV_ref_time[4] = svm->VV_ref_time[3]+Vt[4];
            svm->VV_ref_time[5] = Ts;
            svm->VV_ref_time[6] = Ts;
        }
    
    }

    svm->sector = sector;

    svm->tu = tu;
    svm->tv = tv;
    svm->tw = tw;

    return;
}

