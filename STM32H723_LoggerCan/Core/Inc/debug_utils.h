/*
 * debug_utils.h
 *
 *  Created on: Mar 2, 2025
 *      Author: marco
 */

#ifndef INC_DEBUG_UTILS_H_
#define INC_DEBUG_UTILS_H_


#include <string.h>
#include "timer_utils.h"
#include "can_buffer.h"

extern UART_HandleTypeDef huart1;

extern volatile uint32_t last_report_time;       // Tempo dell'ultimo report

void print_canMsg(CAN_Message *msg);
void Print_Report();
void print_error_cnt();

#endif /* INC_DEBUG_UTILS_H_ */
