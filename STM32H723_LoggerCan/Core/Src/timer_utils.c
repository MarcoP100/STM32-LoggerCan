/*
 * timer_utils.c
 *
 *  Created on: Mar 2, 2025
 *      Author: marco
 */


#include "timer_utils.h"

volatile uint32_t tim2_timestamp = 0;

void TIM2_Init(void) {
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);  // Massima priorità
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);  // Avvia TIM6 con interrupt
}

uint32_t get_timestamp_10us() {

    uint32_t current_overflow;
    uint32_t current_counter;
    uint32_t timestamp;


    // Lettura sicura del timer
    do {
        current_overflow = tim2_timestamp;
        current_counter = __HAL_TIM_GET_COUNTER(&htim2);
    } while (current_overflow != tim2_timestamp);  // Se cambia, ripeti la lettura

    timestamp = current_overflow + current_counter;

    return timestamp;
}


void set_timestamp(void) {
	tim2_timestamp += 4294967295;  // Ogni overflow aggiunge 65536 tick (a 10 µs per tick)
}
