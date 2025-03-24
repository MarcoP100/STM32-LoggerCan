/*
 * spi_comm.c
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */


#include "spi_comm.h"
#include "debug_utils.h"  // per printf()

#define SPI_MSG_LEN 8


extern SPI_HandleTypeDef hspi1;  // o quello che usi
static uint8_t spiTxBuffer[SPI_MSG_LEN];
static uint8_t spiRxBuffer[SPI_MSG_LEN];
static volatile uint8_t spi_ready = 1;
SPI_Packet txPacket;
SPI_Packet rxPacket;


void SPI_Init(void) {
    // (vuoto, se non serve nulla di particolare ora)
}

void SPI_StartReception(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
        				uint16_t Size) {
    //HAL_SPI_Receive_DMA(&hspi1, spiRxBuffer, SPI_MSG_LEN);

	HAL_StatusTypeDef res = HAL_SPI_TransmitReceive_DMA(&hspi, &pTxData, &pRxData, Size);
	 if (res != HAL_OK) {
		 //printf("ERRORE nel riavvio DMA! codice: %d\r\n", res);
	     ;//printf("DMA TX: %d | DMA RX: %d\n", HAL_DMA_GetState(hspi1.hdmatx), HAL_DMA_GetState(hspi1.hdmarx));
	 }else{
		 spi_ready = 0;
	 }

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        SPI_ProcessReceivedData(spiRxBuffer, SPI_MSG_LEN);
        SPI_StartReception();  // restart
    }
}

void SPI_ProcessReceivedData(uint8_t *data, uint16_t len) {
    printf("SPI RX (%d byte): ", len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}

void prepare_spi_data(uint64_t timestamp) {
	for (int i = 0; i < 8; i++) {
        spiTxBuffer[i] = (timestamp >> (8 * i)) & 0xFF;
    }
	//memcpy(spiTxBuffer, "\x01\x02\x03\x04\x05\x06\x07\x08", 8);

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi->Instance == SPI1) {
        spi_ready = 1;

        // Optionally: process received SPI command in spiRxBuffer
    }
}

void SPI_Task_10ms(void) {
    if (spi_ready) {
        uint64_t ts = get_timestamp_10us();  // oppure HAL_GetTick()
        prepare_spi_data(ts);
        SPI_StartReception();  // DMA parte e trasmetterà quando RPi genera clock
    }
    //printf("spi_ready: %lu \n", spi_ready);
}

void sendViaSPI(CAN_Message *messages, uint8_t count) {

	if (spi_ready) {

		    txPacket.header.marker = 0xA5;
		    txPacket.header.messageCount = count;

		    for (int i = 0; i < count; i++) {
		    	txPacket.messages[i] = messages[i];
		    }

		    // TODO: CRC se vuoi
	        SPI_StartReception(&hspi1, (uint8_t*)&txPacket, (uint8_t*)&rxPacket, sizeof(SPIHeader) + count * sizeof(CAN_Message));  // DMA parte e trasmetterà quando RPi genera clock
	    }

}

