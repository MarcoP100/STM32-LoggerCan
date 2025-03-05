/*
 * timer_utils.c
 *
 *  Created on: Mar 2, 2025
 *      Author: marco
 */


#include "timer_utils.h"

volatile uint32_t tim6_timestamp = 0;

void TIM6_Init(void) {
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);  // Massima priorità
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);  // Avvia TIM6 con interrupt
}

uint32_t get_timestamp_10us() {
    static uint32_t last_timestamp = 0;
    uint32_t current_overflow;
    uint32_t current_counter;
    uint32_t timestamp;
    static uint32_t last_counter;

    // Lettura sicura del timer
    do {
        current_overflow = tim6_timestamp;
        current_counter = __HAL_TIM_GET_COUNTER(&htim6);
    } while (current_overflow != tim6_timestamp);  // Se cambia, ripeti la lettura

    timestamp = current_overflow + current_counter;

    if (timestamp < last_timestamp) {
        printf("ERRORE TIMESTAMP: %lu < %lu con counter %lu to %lu (problema di overflow o race condition)\n",
               timestamp, last_timestamp,last_counter,current_counter);
    }
    last_timestamp = timestamp;
    last_counter = current_counter;

    return timestamp;
}


void set_timestamp(void) {
	tim6_timestamp += 65536;  // Ogni overflow aggiunge 65536 tick (a 10 µs per tick)
}
