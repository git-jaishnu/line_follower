/*
 * utis.c
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */


#include "stm32f4xx_hal.h"
#include "utils.h"
#include "main.h"


int sensor_weight[NUM_SENSORS] = {0,-2.5,-1.5,-0.5,0.5,1.5,2.5,0};

uint16_t sensor_min[NUM_SENSORS] = {0,0,0,0,0,0,0,0};
uint16_t sensor_max[NUM_SENSORS] = {4095,4095,4095,4095,4095,4095,4095,4095};
int sensor_bool[NUM_SENSORS];

float Kp = 0.1, Ki = 0, Kd = 0;

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


void processSensors(void)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int range = sensor_max[i] - sensor_min[i];
        if (range <= 0) range = 1;

        int norm = ((int)(sensor_raw[i] - sensor_min[i]) * 1000) / range;
        norm = constrain_int(norm, 0, 1000);


        sensor_value[i] = 1000 - norm;
    }
}





int get_line_error() {
    long weighted_sum = 0;
    for(int i = 0 ; i < 8 ; i++){
    	weighted_sum += sensor_value[i]*sensor_weight[i];
    }


    return (int)weighted_sum;
}



int last_error = 0;
int integral = 0;

int calculate_pid(int error) {
    int P = error;
    integral += error;
    int D = error - last_error;
    last_error = error;

    return (int)(Kp * P + Ki * integral + Kd * D);
}


void drive_motors(int correction) {
    int base_speed = 500;

    int left_speed = base_speed + correction;
    int right_speed = base_speed - correction;

    left_speed = constrain_int(left_speed, 0, 1000);
    right_speed = constrain_int(right_speed, 0, 1000);


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_speed);
}


