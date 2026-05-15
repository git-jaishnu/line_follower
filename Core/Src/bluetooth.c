/*
 * bluetooth.c
 *
 *  Created on: Apr 4, 2026
 *      Author: jaish
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "utils.h"
#include <string.h>
#include "types.h"

#define RX_BUFFER_SIZE 5

void bt_send(char *msg) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

void processBluetoothCommand(char *cmd , int start , PID_Controller pid , Sensor_Array sensor_array ) {

	if (strncmp(cmd, "PID:", 4) == 0) {
		float p, i, d;
		if (sscanf(cmd + 4, "%f,%f,%f", &p, &i, &d) == 3) {
			pid.Kp = p;
			pid.Ki = i;
			pid.Kd = d;
			pid.integral = 0;
			pid.last_error = 0;
		}
	}

	else if (strncmp(cmd, "PARAM:", 6) == 0) {
		char *bs = strstr(cmd, "BS");
		char *ms = strstr(cmd, "MS");
		if (bs)
			sensor_array.base_speed = atoi(bs + 2);

	}


	else if (strcmp(cmd, "START") == 0) {
		start = 1;
	} else if (strcmp(cmd, "STOP") == 0) {
		start = 0;
	}

	else if (strlen(cmd) > 1) {
		char type = cmd[0];
		float val = atof(&cmd[1]);
		if (type == 'P')
			pid.Kp = val;
		else if (type == 'I')
			pid.Ki = val;
		else if (type == 'D')
			pid.Kd = val;
		else if (type == 'B')
			sensor_array.base_speed = (int) val;
		else if (type == 'X')
			start = 0;
		else if (type == 'O')
			start = 1;
	}

}



