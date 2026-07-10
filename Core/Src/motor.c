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

void set_motor_speed(int left_motor, int right_motor, float battery_voltage) {

	uint16_t Right_Forward = GPIO_PIN_14;
	uint16_t Right_Backward = GPIO_PIN_15;
	uint16_t Left_Backward = GPIO_PIN_12;
	uint16_t Left_Forward = GPIO_PIN_13;

	// --- Voltage Compensation ---
	if (battery_voltage < 1.0f) {
		battery_voltage = 12.4f;
	}

	left_motor = (int) ((left_motor * 12.4f) / battery_voltage);
	right_motor = (int) ((right_motor * 12.4f) / battery_voltage);

	left_motor = constrain_int(left_motor, -999, 999);
	right_motor = constrain_int(right_motor, -999, 999);

	// --- LEFT MOTOR LOGIC ---
	if (left_motor > 0) {
		// Forward: IN1 High, IN2 Low
		HAL_GPIO_WritePin(GPIOB, Left_Forward, 1);
		HAL_GPIO_WritePin(GPIOB, Left_Backward, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_motor);
	} else if (left_motor < 0) {
		// Backward: IN1 Low, IN2 High
		HAL_GPIO_WritePin(GPIOB, Left_Forward, 0);
		HAL_GPIO_WritePin(GPIOB, Left_Backward, 1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (left_motor * -1));
	} else {
		// Stop/Coast: IN1 Low, IN2 Low
		HAL_GPIO_WritePin(GPIOB, Left_Forward, 0);
		HAL_GPIO_WritePin(GPIOB, Left_Backward, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}

	// --- RIGHT MOTOR LOGIC ---
	if (right_motor > 0) {
		// Forward: IN1 High, IN2 Low
		HAL_GPIO_WritePin(GPIOB, Right_Forward, 1);
		HAL_GPIO_WritePin(GPIOB, Right_Backward, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, right_motor);
	} else if (right_motor < 0) {
		// Backward: IN1 Low, IN2 High
		HAL_GPIO_WritePin(GPIOB, Right_Forward, 0);
		HAL_GPIO_WritePin(GPIOB, Right_Backward, 1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (right_motor * -1));
	} else {
		// Stop/Coast: IN1 Low, IN2 Low
		HAL_GPIO_WritePin(GPIOB, Right_Forward, 0);
		HAL_GPIO_WritePin(GPIOB, Right_Backward, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}
}

void follow_line(int correction, Sensor_Array *sensor_array) {

	int left_speed = sensor_array->base_speed + correction;
	int right_speed = sensor_array->base_speed - correction;

	set_motor_speed(left_speed, right_speed, battery_voltage(dma_buffer));
}

void swing_turn_left(Sensor_Array *sa, int speed) {

	set_motor_speed(speed, speed, battery_voltage(dma_buffer));
	HAL_Delay(100);

	set_motor_speed(-speed, speed, battery_voltage(dma_buffer));
	HAL_Delay(100);

	while (1) {
		Sync_Sensors(sa);
		processSensors(sa);
		binarizeSensors(sa);
		int count = count_active_sensors(sa) ;

		if ((sa->array[3].on == 1 || sa->array[4].on == 1) && (count <= 2)) {
			break;
		}

		HAL_Delay(5);
	}

	set_motor_speed(speed, -speed, battery_voltage(dma_buffer));
	HAL_Delay(20);

	set_motor_speed(0, 0, battery_voltage(dma_buffer));
}

void swing_turn_right(Sensor_Array *sa, int speed) {
	set_motor_speed(speed, speed, battery_voltage(dma_buffer));

	HAL_Delay(100);
	set_motor_speed(speed, -speed, battery_voltage(dma_buffer));

	HAL_Delay(100);

	while (1) {
		Sync_Sensors(sa);
		processSensors(sa);
		binarizeSensors(sa);
		int count = count_active_sensors(sa) ;

		if ((sa->array[2].on == 1 || sa->array[3].on == 1) && (count <= 2)) {
			break;
		}

		HAL_Delay(5);
	}

	set_motor_speed(-speed, speed, battery_voltage(dma_buffer));
	HAL_Delay(20);

	set_motor_speed(0, 0, battery_voltage(dma_buffer));
}

void shoot_through(Sensor_Array *sa, int speed) {
	set_motor_speed(speed, speed, battery_voltage(dma_buffer));
	HAL_Delay(100);


	while (1) {
		Sync_Sensors(sa);
		processSensors(sa);
		binarizeSensors(sa);
		int count = count_active_sensors(sa) ;

		if ((sa->array[2].on == 1 || sa->array[3].on == 1) && (count <= 2) ) {
			break;
		}

		HAL_Delay(5);
	}

	HAL_Delay(20);

	set_motor_speed(0, 0, battery_voltage(dma_buffer));
}

void handle_junction(Sensor_Array *sa, JunctionType j, int speed) {
	switch (j) {

	case LEFT_JUNCTION:
		swing_turn_left(sa, speed);
		break;

	case RIGHT_JUNCTION:
		swing_turn_right(sa, speed);
		break;

	case T_JUNCTION:
		shoot_through(sa, BASE_SPEED);
		break;

	case CROSS_JUNCTION:
		HAL_Delay(100);
		break;

	case NO_JUNCTION:
		break;
	default:
		break;
	}
}

