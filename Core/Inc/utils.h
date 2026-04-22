/*
 * utils.h
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>


#define RX_BUFFER_SIZE 5
#define NUM_SENSORS 8
#define BASE_SPEED 700
#define SENSOR_THRESHOLD 500
#define TURN_SPEED 500


#define MAX_I = 1;
#define MIN_I = 0;





float constrain_float(float x, float min, float max);
int constrain_int(int x, int min, int max);
float battery_voltage(volatile uint16_t *dma_buffer);









#endif /* INC_UTILS_H_ */
