/*
 * spi_comm.c
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */


#include "spi_comm.h"
#include "debug_utils.h"  // per printf()


extern SPI_HandleTypeDef hspi1;  // o quello che usi
static uint8_t spiTxBuffer[MAX_MESSAGES_PER_BLOCK];
static uint8_t spiRxBuffer[MAX_MESSAGES_PER_BLOCK];
static volatile uint8_t spi_ready = 1;
SPI_Packet txPacket;
SPI_Packet rxPacket;

volatile uint8_t numMsgToSend = 0;
volatile uint8_t idBufferToSend = 0;


void SPI_Init(void) {
    // (vuoto, se non serve nulla di particolare ora)
}

void SPI_StartReception(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
        				uint16_t Size) {
    //HAL_SPI_Receive_DMA(&hspi1, spiRxBuffer, SPI_MSG_LEN);

	HAL_StatusTypeDef res = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size);
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
        //SPI_StartReception();  // restart
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
        uint64_t ts = get_timestamp_us();  // oppure HAL_GetTick()
        prepare_spi_data(ts);
        //SPI_StartReception();  // DMA parte e trasmetterà quando RPi genera clock
    }
    //printf("spi_ready: %lu \n", spi_ready);
}

void sendViaSPI(CAN_Message *messages, uint8_t count) {

	if (spi_ready) {

		    txPacket.header.marker = 0xA5;
		    txPacket.header.messageCount = count;

		    for (int i = 0; i < count; i++) {
		    	convertToSpiFrame(&messages[i], &txPacket.messages[i]);

		    }

		    // TODO: CRC se vuoi
	        SPI_StartReception(
	        		&hspi1,
					(uint8_t*)&txPacket,
					(uint8_t*)&rxPacket,
					sizeof(SPIHeader) + count * sizeof(CAN_Message));  // DMA parte e trasmetterà quando RPi genera clock
	    }

}

void convertToSpiFrame(const CAN_Message *in, CAN_Log_Message *out){
	memcpy(out->data, in->data, 8);
	out->dlc = in->dlc;
	out->flags = in->flags;
	out->id = in->id;
	out->timestamp_us = in->timestamp;
}


void SPI_ProcessCommand() {
    uint8_t cmd = spiRxBuffer[0];

    switch (cmd) {
    case CMD_STATUS: {
        spiTxBuffer[0] = SPI_MARKER;
        spiTxBuffer[1] = numMsgToSend;  // messaggi disponibili nel triplo buffer
        spiTxLength = 2;
        break;
    }

    case CMD_READ: {
    	prepareTxPacket(&canBuffers[idBufferToSend], numMsgToSend, &txPacket);
		spiTxLength = sizeof(SPIHeader) + numMsgToSend * sizeof(CAN_Log_Message);  // per ora senza CRC
		numMsgToSend = 0;
        break;
    }

    default:
        spiTxBuffer[0] = 0xFF;  // errore / comando sconosciuto
        spiTxLength = 1;
        break;
    }

    // Avvia la risposta via DMA
    HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxBuffer, spiRxBuffer, spiTxLength);
}

void prepareTxBuffer(CANBuffer *buf, uint8_t count, SPI_Packet *packet) {
	if (count > MAX_MESSAGES_PER_BLOCK) {
	        count = MAX_MESSAGES_PER_BLOCK;  // protezione base
	    }

	packet->header.marker = 0xA5;
	packet->header.messageCount = count;

    for (int i = 0; i < count; i++) {
        convertToSpiFrame(&buf->messages[i],
        		&packet->messages[i]);
    }
    packet->crc = 0x0000;  // Placeholder (lo aggiungiamo più avanti)
}

