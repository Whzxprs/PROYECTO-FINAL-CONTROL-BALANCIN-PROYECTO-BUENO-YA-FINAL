#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "control.h"


void init_control(control_t *ctrl){
    memset(ctrl, 0, sizeof(control_t));
    ctrl->vd = DEF_vd;
    ctrl->k1 = DEF_k1;
    ctrl->k2 = DEF_k2;
    ctrl->k3 = DEF_k3;
    ctrl->k4 = DEF_k4;
    ctrl->kp = DEF_kp;
    ctrl->kv = DEF_kv;
    //ctrl->Xa = 0.0;
    //ctrl->Yg = 0.0;
    //ctrl->alpha = 0.0;
    ctrl->accelx = 0.15;
    ctrl->angulox = 0.15;
    ctrl->c1 = DEF_c1;
    ctrl->c2 = 1.0-DEF_c1;
    ctrl->t = 0.0;
    ctrl->iTs = 1/Ts;
    ctrl->esc = pi_/(2*ppr*NR);
    ctrl->escs = 127.0/uM;
    //ctrl->omegar = 0.0;
    //ctrl->omegal = 0.0;
    //ctrl->ur = 0.0;
    //ctrl->ul = 0.0;
    //ctrl->v = 0.0;
    //ctrl->alpha_1 = 0.0;
    //ctrl->theta = 0.0;
    //ctrl->thetad = 0.0;
    //ctrl->thetatil = 0.0;
    ctrl->Rasnkm = Ra/(NR*km);
    ctrl->nkm = NR*km;
    ctrl->v2tauM = 2*tauM;
    ctrl->mv=255.0/(2.0*omegaM*R);
    ctrl->mtheta=255.0/(2.0*0.3);
    ctrl->malpha=255.0/(2.0*alphaM);
    ctrl->momega=255.0/(2.0*omegaM);
    ctrl->mu=255.0/(2.0*uM);
}

void calculate_control(control_t *ctrl){
    //ctrl->incl = 19;
    //printf("incl: %f\r\n", ctrl->incl);
    //ctrl->incr = 20;
    //printf("incr: %f\r\n", ctrl->incr);
    //ctrl->Ax = 11;
    //printf("Ax: %f\r\n", ctrl->Ax);
    //ctrl->Gy = 7;
    //printf("Gy: %f\r\n", ctrl->Gy);
    //ctrl->sr = 111;
    //printf("sr: %f\r\n", ctrl->sr);
    //ctrl->sl = 120;
    //printf("sl: %f\r\n", ctrl->sl);
    ctrl->theta = ctrl->sr - ctrl->sl;
    //printf("theta: %f\r\n", ctrl->theta);
    if(ctrl->theta > 160.0)
        ctrl->theta = 160.0;
    else if(ctrl->theta < -160.0)
        ctrl->theta = -160.0;

    //printf("theta: %f\r\n", ctrl->theta);
    ctrl->theta=(-0.3*(ctrl->theta))/160.0;	//Error de seguimiento en radianes
    //printf("theta: %f\r\n", ctrl->theta);
    // Calulo velocidad derecha en radianes.
    ctrl->omegar=(ctrl->incr)*(ctrl->esc)*(ctrl->iTs);
    //printf("omegar: %f\r\n", ctrl->omegar);
    // Calculo velocidad izquierda en radianes.
    ctrl->omegal=(ctrl->incl)*(ctrl->esc)*(ctrl->iTs);
    //printf("omegal: %f\r\n", ctrl->omegal);
    // Los valores del MPU los paso a valores flotantes con sus respectivas escalas.
    ctrl->Xa=-(ctrl->Ax)*accel_factor;//inclinación en rango de -1 a 1
    //printf("Xa: %f\r\n", ctrl->Xa);
    ctrl->Yg=(ctrl->Gy)*gyro_factor;//Yg en grados sobre segundo
    //printf("Yg: %f\r\n", ctrl->Yg);
    ctrl->Yg=deg_2_rad*ctrl->Yg;//Yg en rad sobre segundo
    //printf("Yg: %f\r\n", ctrl->Yg);
// Calculo de filtro complementario
    ctrl->accelx=ctrl->Xa*pi_s2;//inclinación en rango de -pi/2 a pi/2 rad
    //printf("accelx: %f\r\n", ctrl->accelx);
    ctrl->angulox=ctrl->c1*(ctrl->angulox_1+(ctrl->Yg*Ts))+(ctrl->c2*ctrl->accelx);//ecuación del filtro complementario
    //printf("angulox: %f\r\n", ctrl->angulox);
    ctrl->angulox_1=ctrl->angulox;//respaldando valor pasado
    //printf("angulox_1: %f\r\n", ctrl->angulox_1);
    ctrl->alpha=-ctrl->angulox;
    //printf("alpha: %f\r\n", ctrl->alpha);
    ctrl->alpha=ctrl->alpha-calpha;//compensación de alineación de IMU
    //printf("alpha: %f\r\n", ctrl->alpha);
    if(ctrl->alpha>=alphaM)
        ctrl->alpha=alphaM;
    else if(ctrl->alpha<=(-alphaM))
        ctrl->alpha=-alphaM;
    //printf("alpha: %f\r\n", ctrl->alpha);
// Calculo velocidad traslacional
    ctrl->v=(ctrl->omegar+ctrl->omegal)*R/2;	
    //printf("v: %f\r\n", ctrl->v);

    ctrl->thetap=(ctrl->omegar-ctrl->omegal)*R/(2*b);
    //printf("thetap: %f\r\n", ctrl->thetap);
    
    ctrl->thetatil=ctrl->theta-ctrl->thetad;  				// theta tilde
    //printf("thetatil: %f\r\n", ctrl->thetatil);
    ctrl->alphap=(ctrl->alpha-ctrl->alpha_1)*ctrl->iTs;				// alpha punto
    //printf("alphap: %f\r\n", ctrl->alphap);
    ctrl->alpha_1=ctrl->alpha;
    //printf("alpha_1: %f\r\n", ctrl->alpha_1);
    ctrl->vtil=ctrl->v-ctrl->vd;								// v tilde
    //printf("vtil: %f\r\n", ctrl->vtil);
    if((ctrl->intvtil<ctrl->v2tauM)&&(ctrl->intvtil>(-ctrl->v2tauM)))		//Integral de v tilde
        ctrl->intvtil=ctrl->intvtil+(Ts*ctrl->vtil);
    else
    {
        if(ctrl->intvtil>=ctrl->v2tauM)
        ctrl->intvtil=0.95*ctrl->v2tauM;
        else if(ctrl->intvtil<=(-ctrl->v2tauM))
        ctrl->intvtil=-0.95*ctrl->v2tauM; 
    }
    //printf("intvtil: %f\r\n", ctrl->intvtil);
// Esfuerzos de control	
    ctrl->taua=(((-ctrl->kv)*(ctrl->thetap))-((ctrl->kp)*(ctrl->thetatil)))*2*b/R;
    //printf("taua: %f\r\n", ctrl->taua);
    ctrl->u=(ctrl->k1*ctrl->alphap)+(ctrl->k2*ctrl->alpha)+(ctrl->k3*ctrl->vtil)+(ctrl->k4*ctrl->intvtil);
    //printf("u: %f\r\n", ctrl->u);
    if(ctrl->u>=ctrl->v2tauM)
        ctrl->u=ctrl->v2tauM;
    if(ctrl->u<=(-ctrl->v2tauM))
        ctrl->u=-ctrl->v2tauM; 
                    
    //printf("u: %f\r\n", ctrl->u);
// Pares por rueda		
    ctrl->taur=(ctrl->taua+ctrl->u)/2.0;  // par derecho
    //printf("taur: %f\r\n", ctrl->taur);
    ctrl->taul=(-ctrl->taua+ctrl->u)/2.0;  // par izquierdo
    //printf("taul: %f\r\n", ctrl->taul);
                    
// Voltaje rueda izquierda
    ctrl->ul=(ctrl->taul*ctrl->Rasnkm)+(ctrl->nkm*ctrl->omegal);
    //ctrl->ul = (uNM*sin(6.28*ctrl->t));	
    //printf("ul: %f\r\n", ctrl->ul);
    //ctrl->ul=uNM*sinf(0.628*ctrl->t);
    //ctrl->ul=0;
    //ctrl->ul=5.0;
    if(ctrl->ul>=uNM)
        ctrl->ul=uNM;
    else if(ctrl->ul<=(-uNM))
        ctrl->ul=-uNM;

    //printf("ul: %f\r\n", ctrl->ul);
    ctrl->uWl = ctrl->ul*ctrl->escs;
    //printf("uWl: %f\r\n", ctrl->uWl);
    
        
// Voltaje rueda derecha    
    ctrl->ur = (ctrl->taur*ctrl->Rasnkm)+(ctrl->nkm*ctrl->omegar);
    //ctrl->ur = (uNM*sin(6.28*ctrl->t));
    //printf("ur: %f\r\n", ctrl->ur);
    //ctrl->ur=uNM*sin(0.628*ctrl->t);
    //ctrl->ur=0.0;
    //ctrl->ur=2.0;
    if(ctrl->ur >=uNM)
        ctrl->ur = uNM;
    else if(ctrl->ur <= (-uNM))
        ctrl->ur = -uNM;
    //printf("ur: %f\r\n", ctrl->ur);
    ctrl->uWr = ctrl->ur * ctrl->escs;
    //printf("uWr: %f\r\n", ctrl->uWr);
    ctrl->pwmA = abs((int)ctrl->uWl);
    if (ctrl->uWl < 0) {
        ctrl->pwmA += 128;
    }
    ctrl->pwmB = abs((int)ctrl->uWr);
    if (ctrl->uWr < 0) {
        ctrl->pwmB += 128;
    }
    ctrl->vdg=(int8_t)(ctrl->mv*ctrl->vd+127);
    ctrl->vg=(int8_t)(ctrl->mv*ctrl->v+127);
    ctrl->thetag=(int8_t)(ctrl->mtheta*ctrl->theta+127);
    ctrl->alphag=(int8_t)(ctrl->malpha*ctrl->alpha+127);
    ctrl->omegalg=(int8_t)(ctrl->momega*ctrl->omegal+127);
    ctrl->omegarg=(int8_t)(ctrl->momega*ctrl->omegar+127);
    ctrl->ulg=(int8_t)(ctrl->mu*ctrl->ul+127);
    ctrl->urg=(int8_t)(ctrl->mu*ctrl->ur+127);
    ctrl->t += Ts;
    
    //printf("t: %f\r\n", ctrl->t);
}
