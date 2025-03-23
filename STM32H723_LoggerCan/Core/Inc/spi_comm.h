/*
 * spi_comm.h
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */

#ifndef INC_SPI_COMM_H_
#define INC_SPI_COMM_H_

#include "stm32h7xx_hal.h"

void SPI_Init(void);
void SPI_StartReception(void);
void SPI_ProcessReceivedData(uint8_t *data, uint16_t len);
void prepare_spi_data(uint64_t timestamp);
void SPI_Task_10ms(void);

#endif /* INC_SPI_COMM_H_ */
