/*
 * motor.c
 *
 *  Created on: Apr 10, 2026
 *      Author: jaish
 */


#include "main.h"

void set_motor_speed(int left_motor , int right_motor){

	if(left_motor <= 0 && right_motor >= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (-1)*left_motor);


	}

	if(left_motor <= 0 && right_motor <= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (-1)*right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (-1)*left_motor);


	}

	if(left_motor >= 0 && right_motor <= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (-1)*right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);


	}

	if(left_motor >= 0 && right_motor >= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);


	}


}


void follow_line(int correction) {


    int left_speed = BASE_SPEED + correction;
    int right_speed = BASE_SPEED - correction;

    left_speed = constrain_int(left_speed, 0, 1000);
    right_speed = constrain_int(right_speed, 0, 1000);


    set_motor_speed(right_speed, left_speed);
}







