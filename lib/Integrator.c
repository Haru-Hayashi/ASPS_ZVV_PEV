#include "Integrator.h"
#define M_PI 3.14159
// *** prototype declaration (static)*** //

// *** function definition *** //
/**
 * @brief constructor of Integrator
 * 
 * @param u_a  x component of input signal
 * @param u_b  y component of input signal
 * @param y_a  x component of output signal
 * @param y_b  y component of output signal
 */

bool Integral_Construct(Integrator* inte){
    int i;
    for(i=0;i<2;++i){
        inte->u_a[i] = 0.0;
        inte->u_b[i] = 0.0;
        inte->y_a[i] = 0.0;
        inte->y_b[i] = 0.0;
    }
    return 0;
}

/**
 * @brief Calculation of pseudo-integrator
 * 
 * @param inte object
 * @param bandwidth [rad/s] bandwidth of the puseudo integral
 * @param Ts [s] sampling time
 * @param theta_comp [rad/s] Compensation phase
 * 
 */

void Puseudo_integrator(Integrator* inte, float u_a, float u_b, float Ts, float bandwidth, float w_1){
    
     // 位相補正角度の計算
    inte->theta_comp = -atan2(bandwidth, w_1);

    // 入力信号の回転座標系変換
    inte->u_a[0] = u_a * cos(inte->theta_comp) - u_b * sin(inte->theta_comp);
    inte->u_b[0] = u_a * sin(inte->theta_comp) + u_b * cos(inte->theta_comp);

    inte->y_a[1] = inte->y_a[0];
    inte->y_b[1] = inte->y_b[0];

    // 擬似積分計算
    inte->y_a[0] = (inte->y_a[1] + inte->u_a[0]*Ts)/(1.0+Ts*bandwidth);
    inte->y_b[0] = (inte->y_b[1] + inte->u_b[0]*Ts)/(1.0+Ts*bandwidth);

}

bool Integral_modulation_Construct(Integral_mod* inte_mod){
    inte_mod->nrm_vi = 0.0;
    inte_mod->nrm_vi_ref = 0.0;
    inte_mod->theta_vi = 0.0;
    inte_mod->theta_vi_ref = 0.0;
    inte_mod->theta_vi_err = 0.0;
    inte_mod->inte_enable = 0;
    return 0;
}

/**
 * @brief Modulation with voltage integration (ZVV insertion)
 * 
 * @param inte_mod object
 * @param inte_values object
 * @param nrm_vi integral norm
 * @param vi_ref integral reference norm
 * @param theta_vi integral phase
 * @param theta_vi_ref integral reference phase
 * @param theta_vi_err integral phase error
 * @param delta boundary (integral phase error)
 * @param Flag_zero ZVV insertation flag
 * 
 */

void Integral_modulation(Integral_mod* inte_mod, Integral_value inte_values, float delta, bool *Flag_zero){
    
    inte_mod->nrm_vi = sqrt(inte_values.res.y_a[0]*inte_values.res.y_a[0] + inte_values.res.y_b[0]*inte_values.res.y_b[0]);
    inte_mod->nrm_vi_ref = sqrt(inte_values.ref.y_a[0]*inte_values.ref.y_a[0] + inte_values.ref.y_b[0]*inte_values.ref.y_b[0]);

    inte_mod->theta_vi = atan2(inte_values.res.y_b[0], inte_values.res.y_a[0]);
    inte_mod->theta_vi_ref = atan2(inte_values.ref.y_b[0], inte_values.ref.y_a[0]);
    inte_mod->theta_vi_err = inte_mod->theta_vi - inte_mod->theta_vi_ref;

    // 出力の位相が先行して折り返したとき
    if(inte_mod->theta_vi_err < -M_PI){
        inte_mod->theta_vi_err = (inte_mod->theta_vi+2*M_PI)-inte_mod->theta_vi_ref;
    }
    // 指令の位相が先行して折り返したとき
    if(inte_mod->theta_vi_err > M_PI){
        inte_mod->theta_vi_err = inte_mod->theta_vi-(inte_mod->theta_vi_ref+2*M_PI);
    }

    if(inte_mod->theta_vi_err > 0.5*delta){
		*Flag_zero = 1;
	}
	if((*Flag_zero == 1) && (inte_mod->theta_vi_err < -0.5*delta)){
		*Flag_zero = 0;
	}

    if(inte_mod->nrm_vi_ref > 1e-6){
        inte_mod->inte_enable = 1;
    }else{
        inte_mod->inte_enable = 0;
    }

}