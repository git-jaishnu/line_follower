/*
 * utis.c
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */


#include "stm32f4xx_hal.h"
#include "utils.h"


int sensor_weight[NUM_SENSORS] = {-3500,-2500,-1500,-500,500,1500,2500,3500};

uint16_t sensor_min[NUM_SENSORS] = {500,500,500,500,500,500,500,500};
uint16_t sensor_max[NUM_SENSORS] = {3500,3500,3500,3500,3500,3500,3500,3500};

void calibrateSensors(uint16_t sensor_raw[])
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }

    for (int t = 0; t < 3000; t++)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (sensor_raw[i] < sensor_min[i]) sensor_min[i] = sensor_raw[i];
            if (sensor_raw[i] > sensor_max[i]) sensor_max[i] = sensor_raw[i];
        }

        HAL_Delay(1);
    }
}



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





