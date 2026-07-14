/*
 * motor.h
 *
 *  Created on: Apr 10, 2026
 *      Author: jaish
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "types.h"

extern volatile float right_motor_trim;
extern volatile TurnPriority turn_priority;

void follow_line(int correction, int base_speed);
void set_motor_speed(int left_motor , int right_motor , float battery_voltage);
void turn_jugaad(Sensor_Array *sa, int correction, int base_speed);
void jugaad(Sensor_Array *sa, int speed);


#endif /* INC_MOTOR_H_ */
