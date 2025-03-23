/*
 * can_buffer.c
 *
 *  Created on: Mar 1, 2025
 *      Author: marco
 */


#include "can_buffer.h"

CAN_Message canBuffer[CAN_BUFFER_SIZE];
volatile uint8_t canBufferHead = 0;
volatile uint8_t canBufferTail = 0;
volatile uint8_t buffer_count = 0;

volatile uint32_t last_timestamps[10] = {0};  // Ultimi tempi di ricezione
volatile uint16_t anomaly_counts[10] = {0};   // Contatore anomalie
volatile uint32_t msg_counter[10] = {0};
volatile uint32_t max_delta[10] = {0};
volatile uint32_t min_delta[10] = {99999};

volatile uint32_t last_timestamp_buffer = 0;

volatile uint32_t ok_timestamp = 0;
volatile uint32_t last_counter_msg = 0;
volatile uint32_t ok_timestamp_read = 0;

volatile uint32_t durata_interrupt = 0;

volatile uint32_t num_fifo0 = 0;
volatile uint32_t num_fifo1 = 0;

uint32_t RxFifo0ITs = 0;
uint32_t RxFifo1ITs = 0;


void buffer_write(CAN_Message *msg) {

	__disable_irq();

    canBuffer[canBufferHead] = *msg;
    canBufferHead = (canBufferHead + 1) % CAN_BUFFER_SIZE;
    if (buffer_count < CAN_BUFFER_SIZE) {
    	buffer_count++;
    } else {
    	canBufferTail = (canBufferTail + 1) % CAN_BUFFER_SIZE; // Sovrascrive il più vecchio
    }

    ok_timestamp++;

    __enable_irq();

}

int buffer_read(CAN_Message *msg) {
    if (buffer_has_data() == 0) {
        return 0; // Nessun dato disponibile
    }

    /*NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
    NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
    NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
    NVIC_DisableIRQ(FDCAN3_IT1_IRQn);*/


    *msg = canBuffer[canBufferTail];  // Legge il messaggio dal buffer


    canBufferTail = (canBufferTail + 1) % CAN_BUFFER_SIZE; // Avanza il puntatore
    buffer_count--; // Decrementa il contatore

    ok_timestamp_read++;

    /*NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
	NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
	NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
	NVIC_EnableIRQ(FDCAN3_IT1_IRQn);*/

    return 1;


}

int buffer_has_data(void) {
    return buffer_count > 0;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

	__HAL_FDCAN_DISABLE_IT(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);

	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		FDCAN_RxHeaderTypeDef rxHeader;
		uint8_t rxData[8];
		num_fifo0 ++;


		   // Legge il messaggio dalla FIFO 0
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {

			CAN_Message msg;
			msg.id = rxHeader.Identifier;
			msg.dlc = FDCAN_GetRxDataLength(rxHeader.DataLength);
			msg.flags = rxHeader.RxFrameType;
			msg.timestamp = get_timestamp_10us(); // Timestamp in µs

			for (int i = 0; i < 8; i++) {
				msg.data[i] = rxData[i];
			}

			buffer_write(&msg); // Scrive il messaggio nel buffer circolare

		}
	}

		 if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
		        printf("⚠️ Messaggio perso su FIFO0!\n");
		    }

		    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) {
		        printf("⚠️ FIFO0 piena!\n");
		    }
		__HAL_FDCAN_ENABLE_IT(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);



}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {

	__HAL_FDCAN_DISABLE_IT(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

	if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    num_fifo1 ++;


    // Legge il messaggio dalla FIFO 0
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {

    	CAN_Message msg;
    	msg.id = rxHeader.Identifier;
    	msg.dlc = FDCAN_GetRxDataLength(rxHeader.DataLength);
    	msg.flags = rxHeader.RxFrameType;
		msg.timestamp = get_timestamp_10us(); // Timestamp in µs

		for (int i = 0; i < 8; i++) {
			msg.data[i] = rxData[i];
		}

		buffer_write(&msg); // Scrive il messaggio nel buffer circolare

    }
	}
	if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) {
			        printf("⚠️ Messaggio perso su FIFO0!\n");
			    }

			    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) {
			        printf("⚠️ FIFO0 piena!\n");
			    }
    __HAL_FDCAN_ENABLE_IT(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

void test_can_transmit(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};


    TxHeader.Identifier = 0x123;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;



    printf("CAN Mode: %lu\n", hfdcan->Init.Mode);

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        printf("Messaggio CAN inviato con ID: 0x%03lX\n", TxHeader.Identifier);
    } else {
        //printf("Errore nell'invio del messaggio CAN\n");
        uint32_t canError = HAL_FDCAN_GetError(hfdcan);
        printf("CAN Error State: 0x%lX\n", canError);
    }
}


void test_can_loopback(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};


    TxHeader.Identifier = 0x123;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

    //printf("Modalità CAN: %d\n", hcan->Init.Mode);

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        printf("Messaggio CAN inviato in Loopback Mode\n");
    } else {
    	uint32_t canError = HAL_FDCAN_GetError(hfdcan);
    	printf("CAN Error State: 0x%lX\n", canError);
    }

    HAL_Delay(10);

    check_can_rx_polling(hfdcan);

}

void check_can_rx_polling(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    printf("Messaggi in FIFO0: 0x%lX\n", HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0));
    uint32_t canError = HAL_FDCAN_GetError(hfdcan);
    printf("CAN Error State: 0x%lX\n", canError);
    printf("Stato del bus CAN: 0x%d\n", HAL_FDCAN_GetState(hfdcan));

    // Controlliamo se c'è un messaggio disponibile nel FIFO prima di leggerlo
	if ((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) ||
			(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0))	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			printf("Ricevuto CAN (Polling)!\n");
			printf("ID: 0x%03lX DLC: 0x%lX Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				   RxHeader.Identifier, RxHeader.DataLength,
				   RxData[0], RxData[1], RxData[2], RxData[3],
				   RxData[4], RxData[5], RxData[6], RxData[7]);
		} else {
			printf("Errore nella ricezione del messaggio (Polling)!\n");
		}
	} else {
		printf("Nessun messaggio disponibile nel FIFO (Polling)\n");
	}
}

void check_msg_timestamp(CAN_Message *msg){
	uint8_t index = msg->id - CAN_ID_START;
	if (msg->id >= CAN_ID_START && msg->id <= CAN_ID_END) {
		msg_counter[index] ++;
		if (last_timestamps[index] > 0) {
			uint32_t delta_t = msg->timestamp - last_timestamps[index];

			if (delta_t > TIMEOUT_MAX || delta_t < TIMEOUT_MIN) {
				anomaly_counts[index]++;  // Conta l'anomalia
			}
			if (delta_t > max_delta[index]){
				max_delta[index] = delta_t;
			}
			if (delta_t < min_delta[index]){
				min_delta[index] = delta_t;
			}
		}

		last_timestamps[index] = msg->timestamp;
	}
}

uint8_t FDCAN_GetRxDataLength(uint32_t dlc) {
    switch(dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        // Aggiungi altri per CAN FD se vuoi
        default: return 0;
    }
}


