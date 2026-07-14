/*
 * utils.h
 *
 *  Created on: Apr 3, 2026
 *      Author: jaish
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>


#define RX_BUFFER_SIZE 30 // was 5 - didn't match the actual rx_buffer[30] in
                           // main.c, which never referenced this macro anyway.
                           // 5 bytes wouldn't fit a single real command like
                           // "PARAM:BS500;PL300" - now matches reality.
#define NUM_SENSORS 8
#define BASE_SPEED 500
#define SENSOR_THRESHOLD 500
#define TURN_SPEED 500


// Fixed: previously `#define MAX_I = 1;` / `#define MIN_I = 0;` - the `=`
// and trailing `;` get pasted into any expression that uses them, e.g.
// `x > MAX_I` would expand to `x > = 1;`, a syntax error. Unused today, so
// this never bit anyone, but fixed so they're safe if you wire them in
// later (e.g. as PID integral anti-windup bounds).
#define MAX_I 1
#define MIN_I 0





float constrain_float(float x, float min, float max);
int constrain_int(int x, int min, int max);
float battery_voltage(volatile uint16_t *dma_buffer);









#endif /* INC_UTILS_H_ */
