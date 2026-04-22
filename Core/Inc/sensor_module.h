/*
 * sensor_module.h
 *
 *  Created on: Apr 11, 2026
 *      Author: jaish
 */

#ifndef INC_SENSOR_MODULE_H_
#define INC_SENSOR_MODULE_H_



#include "types.h"








extern volatile uint16_t dma_buffer[NUM_SENSORS + 1];

void Initialize_Sensor_Array(Sensor_Array *sensor_array);

void Sync_Sensors(Sensor_Array *sensor_array);
void autoCalibrate(Sensor_Array *sensor_array , uint32_t duration_ms, int speed);

void processSensors(Sensor_Array *sensor_array);
void binarizeSensors(Sensor_Array *sensor_array);


int get_line_error(Sensor_Array *sensor_array);
int get_line_error_digital(Sensor_Array* sensor_array);

int calculate_pid(PID_Controller *pid, int error, float dt);

JunctionType detect_junction(Sensor_Array *sensor_array);
JunctionType detect_junction_digital(Sensor_Array *sensor_array);



#endif /* INC_SENSOR_MODULE_H_ */
