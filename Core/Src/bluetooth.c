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
#include <stdio.h>
#include <stdlib.h>
#include "types.h"

// (RX_BUFFER_SIZE was redefined here, duplicating utils.h's definition -
// removed. It wasn't used in this file anyway.)

void bt_send(char *msg) {
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

void processBluetoothCommand(char *cmd, volatile int *start, PID_Controller *pid, Sensor_Array *sensor_array, volatile int *telemetry_enabled) {

	if (strncmp(cmd, "PID:", 4) == 0) {
		float p, i, d;
		if (sscanf(cmd + 4, "%f,%f,%f", &p, &i, &d) == 3) {
			pid->Kp = p;
			pid->Ki = i;
			pid->Kd = d;
			pid->integral = 0;
			pid->last_error = 0;
		}
	}

	else if (strncmp(cmd, "PARAM:", 6) == 0) {
		char *bs = strstr(cmd, "BS");
		char *pl = strstr(cmd, "PL");
		if (bs)
			sensor_array->base_speed = atoi(bs + 2);
		if (pl)
			pid->limit = atoi(pl + 2);
	}


	else if (strcmp(cmd, "START") == 0) {
		*start = 1;
	} else if (strcmp(cmd, "STOP") == 0) {
		*start = 0;
	}

	// TELEM:0 / TELEM:1 - send from the tuner's raw-command box. Turn this
	// off right before a timed run: at a fixed 9600 baud, each telemetry
	// packet blocks the control loop for ~167ms, which you don't want eating
	// into your loop rate mid-race. Turn it back on when you want to watch
	// live plots while tuning.
	else if (strncmp(cmd, "TELEM:", 6) == 0) {
		*telemetry_enabled = (atoi(cmd + 6) != 0);
	}

	else if (strlen(cmd) > 1) {
		char type = cmd[0];
		float val = atof(&cmd[1]);
		if (type == 'P')
			pid->Kp = val;
		else if (type == 'I')
			pid->Ki = val;
		else if (type == 'D')
			pid->Kd = val;
		else if (type == 'B')
			sensor_array->base_speed = (int) val;
		else if (type == 'X')
			*start = 0;
		else if (type == 'O')
			*start = 1;
		else if (type == 'T')
			*telemetry_enabled = (val != 0);
	}

}
