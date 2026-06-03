#include "stdint.h"
#include "stdio.h"
#include "math.h"

#define DEF_vd 0.1

/* Equivalencias con _TEORIA/variables_fisicas.txt:
 * k1=kdi, k2=kpi, k3=kpv, k4=kiv, kp=kpo, kv=kdo.
 */
#define DEF_k1 0.16
#define DEF_k2 2.20
#define DEF_k3 1.0
#define DEF_k4 2.0


#define DEF_kp 0.09
#define DEF_kv 0.01


#define calpha 0.145//0.2

//Parámetros, rangos y factores
#define tauM	0.1654//0.3//0.1654	//par máximo en rueda [Nm]
#define alphaM	1.4
#define omegaM	44.0
#define uM		11.0
#define uNM		11.0

#define Ts      0.01
#define ppr     12.0
#define pi_     3.141593
#define pi_s2   1.570796
#define Ra      3.0
#define NR      34.014
#define R	    0.035
#define km      0.0008//0.01186
#define Cz	    0.0992//0.0485
#define Mp	    0.3730//0.4310//0.4580
#define b	    0.09

#define deg_2_rad 			(pi_ / 180.0)
#define rad_2_deg 			(180.0 / pi_)
#define accel_div_factor 	16384.0
#define gyro_div_factor 	131.0
#define accel_factor 		6.103515625e-05
#define gyro_factor 		0.007633587786259542

#define DEF_c1 0.995

typedef struct {
    float vd, k1, k2, k3, k4;
    float Xa, Yg, alpha;
    float accelx, angulox, angulox_1, c1, c2;
    float t, iTs, esc, escs;
    float omegar, omegal;
    float kv,kp;
    float sr,sl;
    float incr, incl;
    float Ax, Gy;
    float ur, ul;
    float uWl, uWr;
    float v,alpha_1,theta,thetad,thetatil,taua,u;
    float alphap,vtil,thetap,taur,taul; 
    float Rasnkm,nkm,intvtil,v2tauM;
    int pwmA, pwmB;
    float mv,mtheta,malpha,momega,mu;
    int8_t vdg,vg,thetag,alphag,omegalg,omegarg,ulg,urg;

}control_t;

void init_control(control_t *ctrl);

void calculate_control(control_t *ctrl);
