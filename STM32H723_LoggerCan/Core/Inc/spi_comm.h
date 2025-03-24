/*
 * spi_comm.h
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */

#ifndef INC_SPI_COMM_H_
#define INC_SPI_COMM_H_

#include "stm32h7xx_hal.h"


#define MAX_MESSAGES_PER_BLOCK 64

typedef struct __attribute__((packed)) {
    uint8_t  marker;       // Es: 0xA5 per sync
    uint8_t  messageCount; // Quanti messaggi sono presenti
} SPIHeader;

typedef struct __attribute__((packed)) {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint32_t timestamp_us;
    uint8_t  flags;
} CAN_Message;

typedef struct __attribute__((packed)) {
    SPIHeader header;
    CAN_Message messages[MAX_MESSAGES_PER_BLOCK];  // ma ne usi solo `messageCount`
    // uint16_t crc;  // (opzionale)
} SPI_Packet;

void SPI_Init(void);
void SPI_StartReception(void);
void SPI_ProcessReceivedData(uint8_t *data, uint16_t len);
void prepare_spi_data(uint64_t timestamp);
void SPI_Task_10ms(void);
void sendViaSPI(CAN_Message *messages, uint8_t count);

#endif /* INC_SPI_COMM_H_ */
