/*
 * motor.c
 *
 *  Created on: Apr 10, 2026
 *      Author: jaish
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include  "utils.h"
#include "sensor_module.h"
#include "motor.h"

void set_motor_speed(int left_motor , int right_motor , float battery_voltage){

	left_motor = (left_motor*8.4)/((int)battery_voltage);
	right_motor = (right_motor*8.4)/((int)battery_voltage);

	left_motor  = constrain_int(left_motor,  0, 999);
	right_motor = constrain_int(right_motor, 0, 999);

	if(left_motor <= 0 && right_motor >= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (-1)*left_motor);


	}

	if(left_motor <= 0 && right_motor <= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (-1)*right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (-1)*left_motor);


	}

	if(left_motor >= 0 && right_motor <= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (-1)*right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 999);


	}

	if(left_motor >= 0 && right_motor >= 0){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 999);


	}


}


void follow_line(int correction , Sensor_Array* sensor_array) {


    int left_speed = sensor_array->base_speed + correction;
    int right_speed = sensor_array->base_speed - correction;

    left_speed = constrain_int(left_speed, 0, 999);
    right_speed = constrain_int(right_speed, 0, 999);


    set_motor_speed(right_speed, left_speed , battery_voltage(dma_buffer));
}



void swing_turn_left(Sensor_Array *sa, int speed) {
	int error = -1000;
    set_motor_speed(-speed, speed, battery_voltage(dma_buffer));


    HAL_Delay(100);

    while (error < -250) {
        Sync_Sensors(sa);
        processSensors(sa);
        binarizeSensors(sa);
        error = get_line_error(sa);

        HAL_Delay(5);
    }

    set_motor_speed(0, 0, battery_voltage(dma_buffer));
}

void swing_turn_right(Sensor_Array *sa, int speed) {
	int error = 1000;
    set_motor_speed(speed, -speed, battery_voltage(dma_buffer));

    HAL_Delay(100);

    while (error > 250) {
        Sync_Sensors(sa);
        processSensors(sa);
        binarizeSensors(sa);
        error = get_line_error(sa);

        HAL_Delay(5);
    }

    set_motor_speed(0, 0, battery_voltage(dma_buffer));
}


void handle_junction(Sensor_Array *sa, JunctionType j , int speed) {
    switch (j) {

        case LEFT_JUNCTION:
            swing_turn_left(sa, speed);
            break;

        case RIGHT_JUNCTION:
            swing_turn_right(sa, speed);
            break;

        case T_JUNCTION:
        	HAL_Delay(100);
        	break;

        case CROSS_JUNCTION:
        	HAL_Delay(100);
            break;


        case NO_JUNCTION:
        default:
            break;
    }
}








