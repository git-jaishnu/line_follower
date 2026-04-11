/*
 * sensor_module.h
 *
 *  Created on: Apr 11, 2026
 *      Author: jaish
 */

#ifndef INC_SENSOR_MODULE_H_
#define INC_SENSOR_MODULE_H_




typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float limit;
} PID_Controller;


typedef struct {
	uint16_t adc_raw;
	uint16_t adc_max;
	uint16_t adc_min;
	uint16_t weight;
	int threshold;
	int mapped_value;
	int on;


} Sensor;


typedef struct {
	int number_of_sensors;
	Sensor *array;
	int *weights;

} Sensor_Array;



extern volatile uint16_t dma_buffer[NUM_SENSORS];

extern int sensor_max[NUM_SENSORS];
extern int sensor_min[NUM_SENSORS];
extern int sensor_weight[NUM_SENSORS];
extern int sensor_bool[NUM_SENSORS];



void Sync_Sensors(Sensor_Array *sensor_array);
void autoCalibrate(Sensor_Array *sensor_array , uint32_t duration_ms, int speed);

void processSensors(Sensor_Array *sensor_array);
void binarizeSensors(Sensor_Array *sensor_array);


int get_line_error(Sensor_Array *sensor_array);
int calculate_pid(PID_Controller *pid, int error, float dt);

int detect_right();
int detect_left();


#endif /* INC_SENSOR_MODULE_H_ */
