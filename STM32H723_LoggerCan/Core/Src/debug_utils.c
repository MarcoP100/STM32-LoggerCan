/*
 * usart_utils.c
 *
 *  Created on: Mar 2, 2025
 *      Author: marco
 */


#include "debug_utils.h"
#include <stdio.h>

//volatile uint32_t last_report_time = 0;

void print_canMsg(CAN_Message *msg) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "ID: 0x%03lX DLC: %d Data: %02X %02X %02X %02X %02X %02X %02X %02X Time: %lu us\r\n",
			msg->id, msg->dlc,
			msg->data[0], msg->data[1], msg->data[2], msg->data[3],
			msg->data[4], msg->data[5], msg->data[6], msg->data[7],
			msg->timestamp);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void Print_Report() {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_report_time >= REPORT_INTERVAL) {
        printf("\n=== Report Anomalie CAN ===\n");
        for (uint8_t i = 0; i < 10; i++) {
            printf("ID 0x%X: %d anomalie - %lu ric - %lu min - %lu max\n", CAN_ID_START + i, anomaly_counts[i], msg_counter[i],min_delta[i], max_delta[i] );
            min_delta[i] = 99999;
            max_delta[i] = 0;
        }

        last_report_time = current_time;
    }
}

void print_error_cnt() {
	//uint32_t current_time = HAL_GetTick();
	uint32_t msgPersi = 0;
	    //if (current_time - last_report_time >= REPORT_INTERVAL) {
	        printf("Write:  %lu\n", ok_timestamp);
	        printf("Read:  %lu\n", ok_timestamp_read);
	        uint32_t msgRicevuti = num_fifo0 + num_fifo1;
	        uint32_t msgScritti = ok_timestamp;
	        msgPersi = msgRicevuti - msgScritti;
	        printf("Fifo: %lu / %lu \n", num_fifo0, num_fifo1);
	        printf("Ricevuti: %lu, Scritti: %lu, Persi: %lu \n\n\n\n", msgRicevuti, msgScritti, msgPersi);
	        //printf("Interrupt: %d\n\n\n", durata_interrupt);

	        //last_report_time = current_time;
	        //durata_interrupt = 0;
	    //}
}
