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
#define MESSAGES_DEFAULT 3

#define NO_CMD			0x00
#define CMD_STATUS     	0x01
#define CMD_READ       	0x02
#define CMD_NOT_READY 	0x03
#define CMD_TEST_MODE   0x10
#define CMD_TEST_FILL  	0x11
#define CMD_RESET	 	0xFF


/*** TEST ***/
#define TEST_SINGLE_VALUE  0x01
#define TEST_RAMP          0x02
#define TEST_OVERFLOW      0x03
#define TEST_FILL_READ_BUFFER 0x04

#define SPI_MARKER              0xA5

#define SPI_WATCHDOG_TIMEOUT    200  // 1 secondo

#define SPI_MAX_RETRIES         3




typedef struct __attribute__((packed)) {
    uint8_t  	marker;       // Es: 0xA5 per sync
    uint8_t  	messageCount; // Quanti messaggi sono presenti
    uint8_t 	packetType;    // 0x01 = dati CAN
    uint8_t 	reserved;      // futuro (o CRC header, o sempre 0)
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
void SPI_reStartMode(void);
void SPI_StartReception(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,uint16_t Size);
void SPI_ProcessReceivedData(uint8_t *data, uint16_t len);
void convertToSpiFrame(const CAN_Message *in, CAN_Log_Message *out);
void SPI_ProcessCommand();
bool selectTxBuffer(CANBuffer *buf, uint8_t *spiTxLength);
void prepareTxBuffer(CANBuffer *buf, SPI_Packet *packet, uint8_t reserved);
void prepareTxStatus(SPIHeader *header, uint8_t count, uint8_t reserved);
void prepareNotReadyPacket(SPIHeader *header, uint8_t reserved);
void SPI_SetWatchdog(bool enable);
void SPI_ResetWatchdog();
bool SPI_IsWatchdogExpired();
bool SPI_IsWatchdogEnabled();
bool SPI_IsReady();
void SPI_CS_Pin_Callback(void);
static void handle_testFillReadBuffer(uint8_t numMsg);
bool SPI_IsPacketValid(SPI_Packet *packet);


#endif /* INC_SPI_COMM_H_ */
