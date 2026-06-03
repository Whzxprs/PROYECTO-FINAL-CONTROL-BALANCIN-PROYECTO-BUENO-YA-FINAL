#include "balancin_control.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float lo, float hi)
{
    if (value > hi) {
        return hi;
    }
    if (value < lo) {
        return lo;
    }
    return value;
}

static uint8_t encode_u8(float value)
{
    int encoded = (int)lrintf(value);
    if (encoded > 255) {
        encoded = 255;
    } else if (encoded < 0) {
        encoded = 0;
    }
    return (uint8_t)encoded;
}

void balancin_control_init(balancin_control_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->vd = BAL_VD;
    ctrl->alphad = BAL_ALPHAD;
    ctrl->thetad = BAL_THETAD;
    ctrl->kpi = BAL_KPI;
    ctrl->kdi = BAL_KDI;
    ctrl->kpv = BAL_KPV;
    ctrl->kiv = BAL_KIV;
    ctrl->kpo = BAL_KPO;
    ctrl->kdo = BAL_KDO;

    ctrl->accelx = 0.15f;
    ctrl->angulox = 0.15f;
    ctrl->c1 = BAL_C1;
    ctrl->c2 = 1.0f - BAL_C1;
    ctrl->iTs = 1.0f / BAL_TS;
    ctrl->esc = BAL_PI / (2.0f * BAL_PPR * BAL_NR);
    ctrl->escs = (float)BAL_PWM_MAX / BAL_UM;
    ctrl->Rasnkm = BAL_RA / (BAL_NR * BAL_KM);
    ctrl->nkm = BAL_NR * BAL_KM;
    ctrl->v2tauM = 2.0f * BAL_TAUM;

    ctrl->mv = 255.0f / (2.0f * BAL_OMEGAM * BAL_R);
    ctrl->mtheta = 255.0f / (2.0f * 0.3f);
    ctrl->malpha = 255.0f / (2.0f * BAL_ALPHAM);
    ctrl->momega = 255.0f / (2.0f * BAL_OMEGAM);
    ctrl->mu = 255.0f / (2.0f * BAL_UM);
}

void balancin_control_set_inputs(balancin_control_t *ctrl,
                                 float sl,
                                 float sr,
                                 float ax,
                                 float gy,
                                 float incr,
                                 float incl)
{
    ctrl->sl = sl;
    ctrl->sr = sr;
    ctrl->ax = ax;
    ctrl->gy = gy;
    ctrl->incr = incr;
    ctrl->incl = incl;
}

void balancin_control_step(balancin_control_t *ctrl)
{
    ctrl->theta = ctrl->sr - ctrl->sl;
    ctrl->theta = clampf(ctrl->theta,
                         -BAL_LINE_THETA_FULL_SCALE_COUNTS,
                         BAL_LINE_THETA_FULL_SCALE_COUNTS);
    ctrl->theta = -0.3f * ctrl->theta / BAL_LINE_THETA_FULL_SCALE_COUNTS;

    ctrl->omegar = ctrl->incr * ctrl->esc * ctrl->iTs;
    ctrl->omegal = ctrl->incl * ctrl->esc * ctrl->iTs;

    ctrl->xa = -ctrl->ax * BAL_ACCEL_FACTOR;
    ctrl->yg = ctrl->gy * BAL_GYRO_FACTOR;
    ctrl->yg = BAL_DEG_2_RAD * ctrl->yg;

    ctrl->accelx = ctrl->xa * BAL_PI_S2;
    ctrl->angulox = ctrl->c1 * (ctrl->angulox_1 + ctrl->yg * BAL_TS) +
                    ctrl->c2 * ctrl->accelx;
    ctrl->angulox_1 = ctrl->angulox;

    ctrl->alpha = -ctrl->angulox;
    ctrl->alpha = ctrl->alpha - BAL_CALPHA;
    ctrl->alpha = clampf(ctrl->alpha, -BAL_ALPHAM, BAL_ALPHAM);

    ctrl->v = (ctrl->omegar + ctrl->omegal) * BAL_R / 2.0f;
    ctrl->thetap = (ctrl->omegar - ctrl->omegal) * BAL_R / (2.0f * BAL_B);
    ctrl->eop = -ctrl->thetap;

    ctrl->eo = ctrl->thetad - ctrl->theta;
    ctrl->ei = ctrl->alphad - ctrl->alpha;
    ctrl->eip = (ctrl->ei - ctrl->ei_1) * ctrl->iTs;
    ctrl->ei_1 = ctrl->ei;
    ctrl->ev = ctrl->vd - ctrl->v;

    if ((ctrl->intev < ctrl->v2tauM) && (ctrl->intev > -ctrl->v2tauM)) {
        ctrl->intev = ctrl->intev + BAL_TS * ctrl->ev;
    } else {
        if (ctrl->intev >= ctrl->v2tauM) {
            ctrl->intev = 0.95f * ctrl->v2tauM;
        }
        if (ctrl->intev <= -ctrl->v2tauM) {
            ctrl->intev = -0.95f * ctrl->v2tauM;
        }
    }

    ctrl->taua = (ctrl->kdo * ctrl->eop + ctrl->kpo * ctrl->eo) *
                 2.0f * BAL_B / BAL_R;
    ctrl->u = -ctrl->kpi * ctrl->ei -
              ctrl->kdi * ctrl->eip -
              ctrl->kpv * ctrl->ev -
              ctrl->kiv * ctrl->intev;
    ctrl->u = clampf(ctrl->u, -ctrl->v2tauM, ctrl->v2tauM);

    ctrl->taur = (ctrl->taua + ctrl->u) / 2.0f;
    ctrl->taul = (-ctrl->taua + ctrl->u) / 2.0f;

    ctrl->ul = ctrl->taul * ctrl->Rasnkm + ctrl->nkm * ctrl->omegal;
    ctrl->ul = clampf(ctrl->ul, -BAL_UNM, BAL_UNM);

    ctrl->ur = ctrl->taur * ctrl->Rasnkm + ctrl->nkm * ctrl->omegar;
    ctrl->ur = clampf(ctrl->ur, -BAL_UNM, BAL_UNM);

    ctrl->left_pwm = (int)lrintf(clampf(ctrl->escs * ctrl->ul,
                                        -(float)BAL_PWM_MAX,
                                        (float)BAL_PWM_MAX));
    ctrl->right_pwm = (int)lrintf(clampf(ctrl->escs * ctrl->ur,
                                         -(float)BAL_PWM_MAX,
                                         (float)BAL_PWM_MAX));

    ctrl->t += BAL_TS;
}

void balancin_control_make_legacy_packet(const balancin_control_t *ctrl,
                                         uint8_t packet[BALANCIN_LEGACY_PACKET_SIZE])
{
    packet[0] = 0xAA;
    packet[1] = encode_u8(ctrl->mv * ctrl->vd + 127.0f);
    packet[2] = encode_u8(ctrl->mv * ctrl->v + 127.0f);
    packet[3] = encode_u8(ctrl->mtheta * ctrl->theta + 127.0f);
    packet[4] = encode_u8(ctrl->malpha * ctrl->alpha + 127.0f);
    packet[5] = encode_u8(ctrl->momega * ctrl->omegal + 127.0f);
    packet[6] = encode_u8(ctrl->momega * ctrl->omegar + 127.0f);
    packet[7] = encode_u8(ctrl->mu * ctrl->ul + 127.0f);
    packet[8] = encode_u8(ctrl->mu * ctrl->ur + 127.0f);
}
