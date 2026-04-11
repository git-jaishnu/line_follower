/*
 * sensor_module.c
 *
 *  Created on: Apr 11, 2026
 *      Author: jaish
 */


#include "stm32f4xx_hal.h"
#include "utils.h"

#include "motor.h"
#include "sensor_module.h"




volatile uint16_t dma_buffer[NUM_SENSORS];




void Initialize_Sensor_Array(Sensor_Array *sensor_array){


	for(int i = 0 ; i < sensor_array->number_of_sensors ; i++){
		sensor_array->array[i].weight = sensor_array->weights[i];
		sensor_array->array[i].adc_min = 0;
		sensor_array->array[i].adc_max = 4095;
		sensor_array->array[i].threshold = SENSOR_THRESHOLD;
	}




}

void Sync_Sensors(Sensor_Array *sensor_array){

	sensor_array->array[0].adc_raw = dma_buffer[4];
	sensor_array->array[1].adc_raw = dma_buffer[3];
	sensor_array->array[2].adc_raw = dma_buffer[5];
	sensor_array->array[3].adc_raw = dma_buffer[2];
	sensor_array->array[4].adc_raw = dma_buffer[6];
	sensor_array->array[5].adc_raw = dma_buffer[1];
	sensor_array->array[6].adc_raw = dma_buffer[7];
	sensor_array->array[7].adc_raw = dma_buffer[0];
}



void autoCalibrate(Sensor_Array *sensor_array , uint32_t duration_ms, int speed) {

    for (int i = 0; i < sensor_array->number_of_sensors; i++) {
    	sensor_array->array[i].adc_min = 0;
    	sensor_array->array[i].adc_max = 4096;
    }

    set_motor_speed(-speed, speed);

    uint32_t startTime = HAL_GetTick();


    while (HAL_GetTick() - startTime < duration_ms) {


        for (int i = 0; i < NUM_SENSORS; i++) {
        	if (sensor_array->array[i].adc_raw < sensor_array->array[i].adc_min) sensor_array->array[i].adc_min = sensor_array->array[i].adc_raw;
        	if (sensor_array->array[i].adc_raw > sensor_array->array[i].adc_max) sensor_array->array[i].adc_max = sensor_array->array[i].adc_raw;

        }


        HAL_Delay(1);
    }


    set_motor_speed(0, 0);
}




void processSensors(Sensor_Array *sensor_array)
{
    for (int i = 0; i < sensor_array->number_of_sensors; i++)
    {
        int range = sensor_array->array[i].adc_max - sensor_array->array[i].adc_min;
        if (range <= 0) range = 1;

        int norm = ((int)(sensor_array->array[i].adc_raw - sensor_array->array[i].adc_min) * 1000) / range;
        norm = constrain_int(norm, 0, 1000);


        sensor_array->array[i].mapped_value = 1000 - norm;
    }

}


void binarizeSensors(Sensor_Array *sensor_array)
{

    for (int i = 0; i < sensor_array->number_of_sensors; i++)
    {

        if (sensor_array->array[i].mapped_value > sensor_array->array[i].threshold)
        {
            sensor_array->array[i].on = 1;
        }
        else
        {
        	sensor_array->array[i].on = 0;
        }
    }
}





int get_line_error(Sensor_Array *sensor_array) {
	long adc_sum = 0;
    long weighted_sum = 0;
    for(int i = 0 ; i < 8 ; i++){
    	weighted_sum += (sensor_array->array[i].mapped_value)*(sensor_array->array[i].weight);
    	adc_sum += sensor_array->array[i].mapped_value;
    }

    return (int)(weighted_sum);
}





int calculate_pid(PID_Controller *pid, int error, float dt) {


    float P = pid->Kp * error;

    // Integral with Windup Guard
    pid->integral += (error * dt);
    if (pid->integral > pid->limit) pid->integral = pid->limit;
    if (pid->integral < -pid->limit) pid->integral = -pid->limit;
    float I = pid->Ki * pid->integral;


    float D = pid->Kd * (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = P + I + D;

    if (output > pid->limit) return (int)pid->limit;
    if (output < -pid->limit) return (int)-pid->limit;

    return (int)output;
}


int count_active_sensors(Sensor_Array *sensor_array) {
    int active_count = 0;
    for (int i = 0; i < sensor_array->number_of_sensors; i++) {
        if (sensor_array->array[i].mapped_value > sensor_array->array[i].threshold) {
            active_count++;
        }
    }
    return active_count;
}

int detect_left(Sensor_Array* sensor_array){

	int sum = 0;
	for(int i  = 0 ; i < 4 ; i++){
		if(sensor_array->array[i].on == 1){
			sum++;
		}
	}
	if(sum > 3){
		return 1;
	}
	else return 0;

}


int detect_right(Sensor_Array* sensor_array){

	int sum = 0;
	for(int i  = 7 ; i > 4 ; i--){
		if(sensor_array->array[i].on == 1){
			sum++;
		}
	}
	if(sum > 3){
		return 1;
	}
	else return 0;

}

