#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_
#include <math.h>

// *** header file include *** //
#include <stdbool.h>

// *** macro definition *** //

// *** type definition *** //
/**
 * @brief the order of Integrator
 * 
 */

typedef struct{
    float u_a[2];
    float u_b[2];
    float y_a[2];
    float y_b[2];
    float theta_comp;
    bool enable;
} Integrator;

typedef struct{
    Integrator res;
    Integrator ref;
} Integral_value;

typedef struct{
    float nrm_vi;
    float nrm_vi_ref;
    float theta_vi;
    float theta_vi_ref;
    float theta_vi_err;
    bool inte_enable;
} Integral_mod;


// *** prototype definition *** //
bool Integral_Construct(Integrator* inte);
void Puseudo_integrator(Integrator* inte, float u_a, float u_b, float Ts, float bandwidth, float w_1);
bool Integral_modulation_Construct(Integral_mod* inte_mod);
void Integral_modulation(Integral_mod* inte_mod, Integral_value inte_values, float delta, bool *Flag_zero);
// *** prototype definition (static)*** //

#endif
