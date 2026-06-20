#include "leg_kinematics.hpp"

#include "state_math.hpp"

#include <math.h>

namespace balance::state
{

void Link2Leg(LinkNPodParam *p, const ChassisParam *chassis) noexcept
{
    float xD, yD, xB, yB, BD, A0, B0, C0, xC, yC;

    p->coord[4] = xD = JOINT_DISTANCE + THIGH_LEN * cosf(p->phi4);
    p->coord[5] = yD = THIGH_LEN * sinf(p->phi4);
    p->coord[0] = xB = THIGH_LEN * cosf(p->phi1);
    p->coord[1] = yB = THIGH_LEN * sinf(p->phi1);

    BD = SafeSqrt((xD - xB) * (xD - xB) + (yD - yB) * (yD - yB));
    A0 = 2.0f * CALF_LEN * (xD - xB);
    B0 = 2.0f * CALF_LEN * (yD - yB);
    C0 = BD * BD;

    p->phi2 = 2.0f * atan2f(B0 + SafeSqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    p->coord[2] = xC = xB + CALF_LEN * cosf(p->phi2);
    p->coord[3] = yC = yB + CALF_LEN * sinf(p->phi2);
    p->phi3 = atan2f(yC - yD, xC - xD);

    p->phi0 = atan2f(yC, xC - JOINT_DISTANCE * 0.5f);
    p->leg_len = SafeSqrt((xC - JOINT_DISTANCE * 0.5f) * (xC - JOINT_DISTANCE * 0.5f) + yC * yC);
    p->theta = p->phi0 - 0.5f * BALANCE_PI - chassis->pitch;
    p->height = p->leg_len * cosf(p->theta);

    const float predict_dt = 0.002f;
    float phi1_pred = p->phi1 + p->phi1_w * predict_dt;
    float phi4_pred = p->phi4 + p->phi4_w * predict_dt;

    xD = JOINT_DISTANCE + THIGH_LEN * cosf(phi4_pred);
    yD = THIGH_LEN * sinf(phi4_pred);
    xB = THIGH_LEN * cosf(phi1_pred);
    yB = THIGH_LEN * sinf(phi1_pred);

    BD = SafeSqrt((xD - xB) * (xD - xB) + (yD - yB) * (yD - yB));
    A0 = 2.0f * CALF_LEN * (xD - xB);
    B0 = 2.0f * CALF_LEN * (yD - yB);
    C0 = BD * BD;

    float phi2_pred = 2.0f * atan2f(B0 + SafeSqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    xC = xB + CALF_LEN * cosf(phi2_pred);
    yC = yB + CALF_LEN * sinf(phi2_pred);

    float phi3_pred = atan2f(yC - yD, xC - xD);
    float phi0_pred = atan2f(yC, xC - JOINT_DISTANCE * 0.5f);
    float leg_len_pred = SafeSqrt((xC - JOINT_DISTANCE * 0.5f) * (xC - JOINT_DISTANCE * 0.5f) + yC * yC);

    p->phi2_w = (phi2_pred - p->phi2) / predict_dt;
    p->phi3_w = (phi3_pred - p->phi3) / predict_dt;
    p->phi0_w = (phi0_pred - p->phi0) / predict_dt;
    p->legd = (leg_len_pred - p->leg_len) / predict_dt;

    p->theta_w = (phi0_pred - 0.5f * BALANCE_PI -
                  (chassis->pitch + chassis->pitch_w * predict_dt) -
                  p->theta) / predict_dt;

    p->height_v = p->legd * cosf(p->theta) -
                  p->leg_len * sinf(p->theta) * p->theta_w;
}

} // namespace balance::state
