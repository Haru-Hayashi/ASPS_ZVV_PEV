#include "PI_Controller.h"
#include "math.h"

#define PI 3.14159265358979

void PI_Controller_init(PI_def *PI_Control){
    PI_Control->err[0] = 0.0;
    PI_Control->err[1] = 0.0;
    PI_Control->output[0] = 0.0;
    PI_Control->output[1] = 0.0;
    PI_Control->prop_out = 0.0;
    PI_Control->inte_out = 0.0;
    PI_Control->limit_err_fb_gain = 0.0;
    PI_Control->limit_err_fb = 0.0;

}



void PI_Controller(PI_def *PI_Control, float Kp, float Ki, float Ts, float limit_value, float err_inte1, float *err_inte2){

    *err_inte2 = err_inte1;

    PI_Control->err[0] = PI_Control->reference - PI_Control->feedback;
    PI_Control->err[1] = PI_Control->err[0] - PI_Control->limit_err_fb;

    PI_Control->prop_out = PI_Control->err[0] * Kp;

    *err_inte2 += PI_Control->err[1] * Ts;
    PI_Control->inte_out = *err_inte2 * Ki;

    PI_Control->limit_err_fb_gain = 1/Kp;

    PI_Control->output[0] =  PI_Control->prop_out + PI_Control->inte_out;

    PI_Control->output[1] = PI_Control->output[0];

    if(PI_Control->output[0] > limit_value){
        PI_Control->output[1] = limit_value;
    }
    if(PI_Control->output[0] < 0.0){
        PI_Control->output[1] = 0.0;
    }

    PI_Control->limit_err_fb = (PI_Control->output[0]-PI_Control->output[1])*PI_Control->limit_err_fb_gain;

    // err_inte = PI_Control->err_inte;

    return;
}