/*
 * spi_comm.c
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */


#include "spi_comm.h"
#include "debug_utils.h"  // per printf()


extern SPI_HandleTypeDef hspi1;  // o quello che usi

static uint8_t spiTxStatusBuffer[2];
static uint8_t spiRxBuffer[2];

SPI_Packet txPackets[2];
bool txBufferReady[2];
uint8_t txBufferTransmit = 0;
SPI_Packet rxPacket;

volatile uint8_t numMsgToSend = 0;
uint8_t spiTxLength = 0;


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
	 }

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        //SPI_ProcessReceivedData(spiRxBuffer, SPI_MSG_LEN);
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi->Instance == SPI1) {
        txBufferTransmit = 0;
        SPI_ProcessCommand();
        // Optionally: process received SPI command in spiRxBuffer
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
    	spiTxStatusBuffer[0] = SPI_MARKER;
    	spiTxStatusBuffer[1] = numMsgToSend;  // messaggi disponibili nel triplo buffer
        spiTxLength = 2;
        HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxStatusBuffer, spiRxBuffer, spiTxLength);
        break;
    }

    case CMD_READ: {
    	numMsgToSend = 0;
    	if ((txBufferReady[0]) && (txBufferTransmit == 0)){
    		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&txPackets[0], (uint8_t *)&rxPacket, spiTxLength);
    		txBufferTransmit = 1;
    		txBufferReady[0] = false;
    	}else if ((txBufferReady[1]) && (txBufferTransmit == 0)){
    		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&txPackets[1], (uint8_t *)&rxPacket, spiTxLength);
    		txBufferTransmit = 2;
    		txBufferReady[1] = false;
    	}else{
    		;
    	}
        break;
    }

    default:
    	spiTxStatusBuffer[0] = 0xFF;  // errore / comando sconosciuto
        spiTxLength = 1;
        HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxStatusBuffer, spiRxBuffer, spiTxLength);
        break;
    }

    // Avvia la risposta via DMA

}

bool selectTxBuffer(CANBuffer *buf, uint8_t *spiTxLength) {

	for (int i = 0; i < 2; i++) {
		if (!txBufferReady[i]) {
			prepareTxBuffer(buf, &txPackets[i]);
			numMsgToSend = buf->index;
			*spiTxLength = sizeof(SPIHeader) + numMsgToSend * sizeof(CAN_Log_Message);
			txBufferReady[i] = true;
			return true;
		}
	}
	return false;

}


void prepareTxBuffer(CANBuffer *buf, SPI_Packet *packet) {

	uint8_t count = buf->index;
	if (count > MAX_MESSAGES_PER_BLOCK)
		count = MAX_MESSAGES_PER_BLOCK;

	packet->header.marker = 0xA5;
	packet->header.messageCount = count;

	for (int i = 0; i < count; i++) {
		convertToSpiFrame(&buf->messages[i],
					&packet->messages[i]);
	}
	packet->crc = 0x0000;  // Placeholder (lo aggiungiamo più avanti)
}

