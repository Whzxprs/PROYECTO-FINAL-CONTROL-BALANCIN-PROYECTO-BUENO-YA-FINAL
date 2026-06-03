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

static void put_u16_le(uint8_t *packet, size_t *offset, uint16_t value)
{
    packet[*offset] = (uint8_t)(value & 0xFF);
    (*offset)++;
    packet[*offset] = (uint8_t)((value >> 8) & 0xFF);
    (*offset)++;
}

static void put_f32_le(uint8_t *packet, size_t *offset, float value)
{
    memcpy(&packet[*offset], &value, sizeof(value));
    *offset += sizeof(value);
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
    ctrl->calpha = BAL_CALPHA;

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
    ctrl->alpha = ctrl->alpha - ctrl->calpha;
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

    if (ctrl->test_ul != 0.0f) {
        ctrl->ul = clampf(ctrl->test_ul, -BAL_UNM, BAL_UNM);
    }
    if (ctrl->test_ur != 0.0f) {
        ctrl->ur = clampf(ctrl->test_ur, -BAL_UNM, BAL_UNM);
    }

    ctrl->left_pwm = (int)lrintf(clampf(ctrl->escs * ctrl->ul,
                                        -(float)BAL_PWM_MAX,
                                        (float)BAL_PWM_MAX));
    ctrl->right_pwm = (int)lrintf(clampf(ctrl->escs * ctrl->ur,
                                         -(float)BAL_PWM_MAX,
                                         (float)BAL_PWM_MAX));

    ctrl->t += BAL_TS;
}

bool balancin_control_apply_command(balancin_control_t *ctrl,
                                    uint8_t cmd_id,
                                    float value)
{
    if (!isfinite(value)) {
        return false;
    }

    switch (cmd_id) {
    case 0:
        ctrl->kpi = value;
        break;
    case 1:
        ctrl->kdi = value;
        break;
    case 2:
        ctrl->kpv = value;
        break;
    case 3:
        ctrl->kiv = value;
        break;
    case 4:
        ctrl->kpo = value;
        break;
    case 5:
        ctrl->kdo = value;
        break;
    case 6:
        ctrl->vd = value;
        break;
    case 7:
        break;
    case 8:
        ctrl->alphad = value;
        break;
    case 9:
        ctrl->c1 = clampf(value, 0.0f, 1.0f);
        ctrl->c2 = 1.0f - ctrl->c1;
        break;
    case 10:
        ctrl->calpha = value;
        break;
    case 11:
        break;
    case 12:
        ctrl->test_ul = clampf(value, -BAL_UNM, BAL_UNM);
        break;
    case 13:
        ctrl->test_ur = clampf(value, -BAL_UNM, BAL_UNM);
        break;
    default:
        return false;
    }

    return true;
}

void balancin_control_make_telemetry_v2_packet(
    const balancin_control_t *ctrl,
    uint16_t sl_raw,
    uint16_t sr_raw,
    uint16_t seq,
    uint8_t flags,
    uint8_t packet[BALANCIN_TELEMETRY_PACKET_V2_SIZE])
{
    size_t offset = 0;

    packet[offset++] = 0xAB;
    packet[offset++] = BALANCIN_TELEMETRY_VERSION;
    put_u16_le(packet, &offset, seq);
    put_f32_le(packet, &offset, ctrl->vd);
    put_f32_le(packet, &offset, ctrl->v);
    put_f32_le(packet, &offset, ctrl->theta);
    put_f32_le(packet, &offset, ctrl->alpha);
    put_f32_le(packet, &offset, ctrl->omegal);
    put_f32_le(packet, &offset, ctrl->omegar);
    put_f32_le(packet, &offset, ctrl->ul);
    put_f32_le(packet, &offset, ctrl->ur);
    put_u16_le(packet, &offset, sl_raw);
    put_u16_le(packet, &offset, sr_raw);
    packet[offset++] = flags;
}
