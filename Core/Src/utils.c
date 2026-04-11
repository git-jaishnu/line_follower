/*
 * utils.c
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */


#include "utils.h"


int constrain_int(int x, int min, int max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float constrain_float(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}




