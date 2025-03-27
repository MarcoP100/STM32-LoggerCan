/*
 * spi_comm.h
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */

#ifndef INC_SPI_COMM_H_
#define INC_SPI_COMM_H_

#include "stm32h7xx_hal.h"
#include "can_buffer.h"


#define MAX_MESSAGES_PER_BLOCK 64

#define CMD_STATUS     0x01
#define CMD_READ       0x02
#define CMD_SEND_CAN   0x10

#define SPI_MARKER              0xA5


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
} CAN_Log_Message;

typedef struct __attribute__((packed)) {
    SPIHeader header;
    CAN_Log_Message messages[MAX_MESSAGES_PER_BLOCK];  // ma ne usi solo `messageCount`
    uint16_t crc;  // (opzionale)
} SPI_Packet;

extern volatile uint8_t numMsgToSend;
extern uint8_t spiTxLength;

void SPI_Init(void);
void SPI_StartReception(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,uint16_t Size);
void SPI_ProcessReceivedData(uint8_t *data, uint16_t len);
void convertToSpiFrame(const CAN_Message *in, CAN_Log_Message *out);
void SPI_ProcessCommand();
bool selectTxBuffer(CANBuffer *buf, uint8_t *spiTxLength);
void prepareTxBuffer(CANBuffer *buf, SPI_Packet *packet);

#endif /* INC_SPI_COMM_H_ */
