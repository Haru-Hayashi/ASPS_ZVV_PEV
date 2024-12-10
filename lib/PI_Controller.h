#ifndef PI_CONTROLLER
#define PI_CONTROLLER


typedef struct{
float err[2];
float reference;
double feedback;
double limit_err_fb;
double limit_err_fb_gain;

double prop_out;
double inte_out;
double err_inte;
double output[2];

} PI_def;

void PI_Controller_init(PI_def *PI_Control);

void PI_Controller(PI_def *PI_Control, float Kp, float Ki, float Ts, float limit_value, float err_inte1, float *err_inte2);

#endif