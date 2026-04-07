/*
 * utils.h
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#define NUM_SENSORS 8

extern int sensor_weight[NUM_SENSORS];

extern float Kp , Ki , Kd;

extern uint16_t sensor_max[NUM_SENSORS];
extern uint16_t sensor_min[NUM_SENSORS];


float constrain_float(float x, float min, float max);
int constrain_int(int x, int min, int max);
void calibrateSensors(uint16_t sensor_raw[]);
void processSensors(void);


int get_line_error();
int calculate_pid(int error);
void drive_motors(int correction);





#endif /* INC_UTILS_H_ */
