/*
 * motor.h
 *
 *  Created on: Apr 10, 2026
 *      Author: jaish
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "types.h"






void follow_line(int correction , Sensor_Array* sensor_array);
void set_motor_speed(int left_motor , int right_motor , float battery_voltage);
void swing_turn_right(Sensor_Array *sa, int speed);
void swing_turn_left(Sensor_Array *sa, int speed);

void handle_junction(Sensor_Array *sa, JunctionType j , int speed);


#endif /* INC_MOTOR_H_ */
