#include "pll.h"

struct PLL_HandleTypeDef {
    PLL_InitTypeDef Init;
    float InputValue;
};