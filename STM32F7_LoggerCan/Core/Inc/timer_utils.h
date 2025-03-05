/*
 * timer_utils.h
 *
 *  Created on: Mar 2, 2025
 *      Author: marco
 */

#ifndef INC_TIMER_UTILS_H_
#define INC_TIMER_UTILS_H_

#include "stm32f7xx_hal.h"

extern TIM_HandleTypeDef htim6;

void TIM6_Init(void);
uint32_t get_timestamp_10us(void);
void set_timestamp(void);


#endif /* INC_TIMER_UTILS_H_ */
