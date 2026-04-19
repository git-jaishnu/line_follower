/*
 * utils.c
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */


#include "utils.h"
#include "sensor_module.h"



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


float battery_voltage(volatile uint16_t *dma_buffer){
	uint16_t adc = dma_buffer[NUM_SENSORS];
	float voltage = (((float)adc) * 3.3f / 4095.0f) / 0.3125f;
	return voltage;

}




