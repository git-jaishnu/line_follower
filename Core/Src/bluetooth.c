/*
 * bluetooth.c
 *
 *  Created on: Apr 4, 2026
 *      Author: jaish
 */

#include "main.h"
#include "utils.h"
#include <stdlib.h>

#define RX_BUFFER_SIZE 5

void bt_send(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

int processBluetoothCommand(uint8_t *comm){

//	char* endptr;
//	float f = strtof(comm, &endptr);
//
//	if(endptr == "P\0"){
//		Kp = f;
//	}
//	else if(endptr == "i"){
//		Ki = f;
//	}else if(endptr == "d"){
//		Kd =f ;
//	}

	bt_send("done");




}

