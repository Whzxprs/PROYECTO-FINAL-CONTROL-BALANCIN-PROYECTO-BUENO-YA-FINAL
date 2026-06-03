#pragma once

#include <stddef.h>
#include <stdint.h>

#define BALANCIN_CONTROL_PERIOD_MS 10
#define BALANCIN_LEGACY_PACKET_SIZE 9

/* References from _TEORIA/variables_fisicas.txt. */
#define BAL_VD     0.1f
#define BAL_ALPHAD 0.0f
#define BAL_THETAD 0.0f

/* Gains from _TEORIA/variables_fisicas.txt. */
#define BAL_KPI 2.20f
#define BAL_KDI 0.16f
#define BAL_KPV 1.0f
#define BAL_KIV 2.0f
#define BAL_KPO 0.09f
#define BAL_KDO 0.01f

/* Physical parameters and sensor factors from _TEORIA/variables_fisicas.txt. */
#define BAL_TS      0.01f
#define BAL_PPR     12.0f
#define BAL_PI      3.141593f
#define BAL_PI_S2   1.570796f
#define BAL_RA      3.0f
#define BAL_NR      34.014f
#define BAL_R       0.035f
#define BAL_KM      0.0008f
#define BAL_B       0.09f
#define BAL_CALPHA  0.145f
#define BAL_TAUM    0.1654f
#define BAL_ALPHAM  1.4f
#define BAL_OMEGAM  44.0f
#define BAL_UM      11.0f
#define BAL_UNM     11.0f

#define BAL_DEG_2_RAD      0.0174533f
#define BAL_RAD_2_DEG      57.29578f
#define BAL_ACCEL_FACTOR   0.0000610352f
#define BAL_GYRO_FACTOR    0.0076336f
#define BAL_C1             0.995f
#define BAL_PWM_MAX        255

/*
 * Line sensors are sampled by ESP32 ADC1 at 12 bits, but the control uses
 * only the 8 most significant bits: adc8 = adc12 >> 4.
 */
#define BAL_LINE_ADC_BITS  8
#define BAL_LINE_ADC_MAX   255.0f
#define BAL_LINE_THETA_FULL_SCALE_COUNTS 160.0f

typedef struct {
    float vd;
    float alphad;
    float thetad;
    float kpi;
    float kdi;
    float kpv;
    float kiv;
    float kpo;
    float kdo;

    float sl;
    float sr;
    float ax;
    float gy;
    float incr;
    float incl;

    float xa;
    float yg;
    float alpha;
    float accelx;
    float angulox;
    float angulox_1;
    float c1;
    float c2;

    float t;
    float iTs;
    float esc;
    float escs;
    float omegar;
    float omegal;
    float ur;
    float ul;
    float v;
    float ei;
    float ei_1;
    float theta;
    float eo;
    float eop;
    float taua;
    float u;
    float eip;
    float ev;
    float thetap;
    float taur;
    float taul;
    float Rasnkm;
    float nkm;
    float intev;
    float v2tauM;

    float mv;
    float mtheta;
    float malpha;
    float momega;
    float mu;

    int left_pwm;
    int right_pwm;
} balancin_control_t;

void balancin_control_init(balancin_control_t *ctrl);
void balancin_control_set_inputs(balancin_control_t *ctrl,
                                 float sl,
                                 float sr,
                                 float ax,
                                 float gy,
                                 float incr,
                                 float incl);
void balancin_control_step(balancin_control_t *ctrl);
void balancin_control_make_legacy_packet(const balancin_control_t *ctrl,
                                         uint8_t packet[BALANCIN_LEGACY_PACKET_SIZE]);
