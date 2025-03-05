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
volatile uint16_t error_timestamp = 0;
volatile uint32_t ok_timestamp = 0;
volatile uint32_t last_counter_msg = 0;

volatile uint32_t last_timestamp_buffer_read = 0;
volatile uint16_t error_timestamp_read = 0;
volatile uint32_t ok_timestamp_read = 0;

volatile uint32_t durata_interrupt = 0;

volatile uint32_t num_fifo0 = 0;
volatile uint32_t num_fifo1 = 0;

void buffer_write(CAN_Message *msg) {
    canBuffer[canBufferHead] = *msg;
    canBufferHead = (canBufferHead + 1) % CAN_BUFFER_SIZE;
    if (buffer_count < CAN_BUFFER_SIZE) {
    	buffer_count++;
    } else {

        canBufferTail = (canBufferTail + 1) % CAN_BUFFER_SIZE; // Sovrascrive il più vecchio
    }
    if (msg->timestamp < last_timestamp_buffer){
    	error_timestamp ++;
    	printf("ERRORE SCRITTURA: ID 0x%X - TS: %lu < %lu (Prev) - cnt; %lu to %lu\n",
    	msg->id, msg->timestamp, last_timestamp_buffer, last_counter_msg, msg->counter);
    }else 	ok_timestamp++;

    last_timestamp_buffer = msg->timestamp;
    last_counter_msg = msg->counter;
}

int buffer_read(CAN_Message *msg) {
    if (buffer_has_data() == 0) {
        return 0; // Nessun dato disponibile
    }

    *msg = canBuffer[canBufferTail];  // Legge il messaggio dal buffer
    canBufferTail = (canBufferTail + 1) % CAN_BUFFER_SIZE; // Avanza il puntatore
    buffer_count--; // Decrementa il contatore

    if (msg->timestamp < last_timestamp_buffer_read){
        	error_timestamp_read ++;
        }else 	ok_timestamp_read++;

        last_timestamp_buffer_read = msg->timestamp;

    //check_msg_timestamp(msg);
    return 1;
}

int buffer_has_data(void) {
    return buffer_count > 0;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    //uint32_t inizio_interrupt = get_timestamp_10us();
    num_fifo0 ++;

    // Disabilita temporaneamente l'interrupt FIFO1 mentre gestiamo FIFO0
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    // Legge il messaggio dalla FIFO 0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {

    	CAN_Message msg;
		msg.id = rxHeader.StdId;
		msg.dlc = rxHeader.DLC;
		msg.flags = rxHeader.RTR;
		msg.timestamp = get_timestamp_10us(); // Timestamp in µs
		msg.counter ++;

		for (int i = 0; i < 8; i++) {
			msg.data[i] = rxData[i];
		}

		buffer_write(&msg); // Scrive il messaggio nel buffer circolare



		/*uint32_t fine_interrupt = get_timestamp_10us();

		uint32_t tmp_durata = fine_interrupt - inizio_interrupt;

		if (tmp_durata > durata_interrupt)
		durata_interrupt = tmp_durata;*/
        // Notifica il task FreeRTOS per elaborare il messaggio
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//vTaskNotifyGiveFromISR(Task_CANHandle, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    // Riabilita l'interrupt FIFO1 dopo la gestione di FIFO0
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    num_fifo1 ++;

    // Disabilita temporaneamente l'interrupt FIFO1 mentre gestiamo FIFO0
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    // Legge il messaggio dalla FIFO 0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {

    	CAN_Message msg;
		msg.id = rxHeader.StdId;
		msg.dlc = rxHeader.DLC;
		msg.flags = rxHeader.RTR;
		msg.timestamp = get_timestamp_10us(); // Timestamp in µs
		msg.counter ++;

		for (int i = 0; i < 8; i++) {
			msg.data[i] = rxData[i];
		}

		buffer_write(&msg); // Scrive il messaggio nel buffer circolare

        // Notifica il task FreeRTOS per elaborare il messaggio
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//vTaskNotifyGiveFromISR(Task_CANHandle, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    // Riabilita l'interrupt FIFO1 dopo la gestione di FIFO0
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

}

/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { // solo per test in loopback mode
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        printf("Messaggio CAN ricevuto!\n");
        printf("ID: 0x%03lX DLC: 0x%03lX Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
               RxHeader.StdId, RxHeader.DLC,
               RxData[0], RxData[1], RxData[2], RxData[3],
               RxData[4], RxData[5], RxData[6], RxData[7]);
    } else {
        printf("Errore nella ricezione CAN\n");
    }
}*/


void test_can_transmit(CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint32_t TxMailbox;

    TxHeader.StdId = 0x123;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

    printf("CAN Mode: %lu\n", hcan->Init.Mode);

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
        printf("Messaggio CAN inviato con ID: 0x%03lX\n", TxHeader.StdId);
    } else {
        //printf("Errore nell'invio del messaggio CAN\n");
        uint32_t canError = HAL_CAN_GetError(hcan);
        printf("CAN Error State: 0x%lX\n", canError);
    }
}


void test_can_loopback(CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint32_t TxMailbox;

    TxHeader.StdId = 0x123;
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;

    //printf("Modalità CAN: %d\n", hcan->Init.Mode);

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
        printf("Messaggio CAN inviato in Loopback Mode\n");
    } else {
    	uint32_t canError = HAL_CAN_GetError(hcan);
    	printf("CAN Error State: 0x%lX\n", canError);
    }

    HAL_Delay(10);

    check_can_rx_polling(hcan);

}

void check_can_rx_polling(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    printf("Messaggi in FIFO0: 0x%lX\n", HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0));
    uint32_t canError = HAL_CAN_GetError(hcan);
    printf("CAN Error State: 0x%lX\n", canError);
    printf("Stato del bus CAN: 0x%lX\n", HAL_CAN_GetState(hcan));

    // Controlliamo se c'è un messaggio disponibile nel FIFO prima di leggerlo
	if ((HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) ||
			(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0))	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			printf("Ricevuto CAN (Polling)!\n");
			printf("ID: 0x%03lX DLC: 0x%lX Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				   RxHeader.StdId, RxHeader.DLC,
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

