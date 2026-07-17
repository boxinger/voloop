#include "voloop.h"

int main(void) {
    PID_HandleTypeDef pid = { 0 };
    FOF_HandleTypeDef fof = { 0 };
    QPR_HandleTypeDef qpr = { 0 };
    NCO_HandleTypeDef nco = { 0 };
    Buck_HandleTypeDef buck = { 0 };
    PLL_HandleTypeDef pll = { 0 };
    OffInv_HandleTypeDef offinv = { 0 };
    PFC_HandleTypeDef pfc = { 0 };

    (void)VOLOOP_DEF_ClampFloat(0.0f, -1.0f, 1.0f);

    (void)VOLOOP_PID_GetState(&pid);
    (void)VOLOOP_FOF_Compute(&fof, 0.0f);
    (void)VOLOOP_QPR_GetState(&qpr);
    (void)VOLOOP_NCO_GetState(&nco);
    (void)VOLOOP_Buck_GetState(&buck);
    (void)VOLOOP_PLL_GetState(&pll);
    (void)VOLOOP_OffInv_GetState(&offinv);
    (void)VOLOOP_PFC_GetState(&pfc);

    return 0;
}
