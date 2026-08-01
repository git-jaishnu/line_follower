/*
 * bluetooth.h
 *
 *  Created on: Apr 4, 2026
 *      Author: jaish
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#define TELEM_INTERVAL_MS 200

void bt_send(char *msg);
void processBluetoothCommand(char *cmd , int start , PID_Controller pid , Sensor_Array sensor_array );



#endif /* INC_BLUETOOTH_H_ */
