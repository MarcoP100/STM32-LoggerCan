/*
 * spi_comm.c
 *
 *  Created on: Mar 22, 2025
 *      Author: marco
 */


#include "spi_comm.h"
#include "debug_utils.h"  // per printf()
#include "main.h"


extern SPI_HandleTypeDef hspi1;  // o quello che usi

static SPIHeader spiTxBuffer;

SPI_Packet txPackets[2];
bool txBufferReady[2];
uint8_t txBufferTransmit = 0;
SPI_Packet spiRxPacket;

volatile uint8_t numMsgToSend = 0;
uint8_t spiTxLength = 0;
static uint32_t lastSpiActivity = 0;
static bool enableWatchdog = false;
static uint8_t spiWatchdogRetries = 0;

static void handle_cmd_status(uint8_t reserved);
static void handle_cmd_read(void);
static void handle_cmd_default(void);
static void handle_cmd_testMode(void);

static bool waitForSpiReady(uint32_t timeoutMs);


//per test
static uint8_t fake_counter = 0;
static bool ascending = true;
SPI_Packet testRxPacket;
SPI_Packet testTxPacket;
//static uint32_t start_time = 0;
//static uint32_t stop_time = 0;


void SPI_Init(void) {
	handle_cmd_default();


}

void SPI_reStartMode() {
	//printf("[SPI] Watchdog: tentativo di recovery\n");
	//start_time = get_timestamp_us();
	if (HAL_SPI_Abort_IT(&hspi1) != HAL_OK) {
		printf("⚠️ SPI Abort fallito\n");
		// Puoi forzare un reset completo con DeInit/Init se vuoi
	}
	if (!waitForSpiReady(100)) {
		// Errore: la SPI non è tornata disponibile
		printf("❌ Timeout SPI ready dopo abort!\n");
		return;
	}

	SPI_Init();
	//stop_time = get_timestamp_us();
    //printf("[SPI] Re-inizializzazione SPI in %lu us\n", stop_time - start_time);

}

static bool waitForSpiReady(uint32_t timeoutMs) {
    uint32_t start = HAL_GetTick();
    while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) &&
    		(HAL_DMA_GetState(hspi1.hdmatx) != HAL_DMA_STATE_READY) &&
    		(HAL_DMA_GetState(hspi1.hdmarx) != HAL_DMA_STATE_READY)) {
        if ((HAL_GetTick() - start) > timeoutMs) {
            return false;  // timeout
        }
    }
    return true;
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

	const SPIHeader* header = (const SPIHeader*)&spiRxPacket;
	uint8_t cmd = isValidPacket(&spiRxPacket) ? header->packetType : CMD_RESET;


	switch (cmd) {

		case CMD_TEST_MODE:

			handle_cmd_testMode();
			break;

		case CMD_TEST_FILL:
			handle_cmd_FillBuffer();
			break;

		case CMD_STATUS:
			handle_cmd_status(0xff);
			break;

		case CMD_READ:
			handle_cmd_read();
			break;

		case NO_CMD:
			//printf("Comando 0: %02X\n", cmd);
			handle_cmd_default();
			break;

	    case CMD_RESET:
	    	SPI_reStartMode();
	    	break;

    default:

    	//printf("Comando sconosciuto: %02X\n", cmd);
    	SPI_reStartMode();
        break;
    }

    // Avvia la risposta via DMA

}


bool selectTxBuffer(CANBuffer *buf, uint8_t *spiTxLength) {

	for (int i = 0; i < 2; i++) {
		if (!txBufferReady[i]) {
			prepareTxBuffer(buf, &txPackets[i], 0xff);
			numMsgToSend = buf->index;
			*spiTxLength = sizeof(SPIHeader) + numMsgToSend * sizeof(CAN_Log_Message);
			txBufferReady[i] = true;
			return true;
		}
	}
	return false;

}

void prepareTxStatus(SPIHeader *header, uint8_t count, uint8_t reserved){

	header->marker = SPI_MARKER;
	header->messageCount = count;
	header->packetType = CMD_STATUS;         // tipo = CAN data
	header->reserved = reserved;

}

void prepareTxBuffer(CANBuffer *buf, SPI_Packet *packet, uint8_t reserved) {

	uint8_t count = buf->index;
	if (count > MAX_MESSAGES_PER_BLOCK)
		count = MAX_MESSAGES_PER_BLOCK;

	packet->header.marker = SPI_MARKER;
	packet->header.messageCount = count;
	packet->header.packetType = CMD_READ;         // tipo = CAN data
	packet->header.reserved = reserved;

	for (int i = 0; i < count; i++) {
		convertToSpiFrame(&buf->messages[i],
					&packet->messages[i]);
	}
	packet->crc = 0x0000;  // Placeholder (lo aggiungiamo più avanti)
}

void prepareNotReadyPacket(SPIHeader *header, uint8_t reserved) {
    header->marker = SPI_MARKER;
    header->messageCount = 0;
    header->packetType = CMD_NOT_READY;
    header->reserved = reserved;
}

void SPI_SetWatchdog(bool enable) {
	enableWatchdog = enable;
}
void SPI_ResetWatchdog() {
	spiWatchdogRetries = 0;
	lastSpiActivity = HAL_GetTick();

}
bool SPI_IsWatchdogExpired() {
	if (SPI_IsWatchdogEnabled()) {
		uint32_t now = HAL_GetTick();
		if ((now - lastSpiActivity) > SPI_WATCHDOG_TIMEOUT) {
			if (HAL_GPIO_ReadPin(SPI_CS_GPIO_Port, SPI_CS_Pin) == GPIO_PIN_SET) {
				// CS è ancora alto: SPI inattiva
				spiWatchdogRetries++;
				if (spiWatchdogRetries >= SPI_MAX_RETRIES) {
					return true;  // timeout effettivo
				} else {
					// Riproviamo a breve, resettiamo il timer
					lastSpiActivity = HAL_GetTick();
				}
			} else {
				// CS basso: master ha iniziato nuova comunicazione, non abortire
				lastSpiActivity = HAL_GetTick();
			}
		}
	}
	return false;
}

bool SPI_IsWatchdogEnabled() {
	return enableWatchdog;
}

bool SPI_IsReady() {
	return (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY);
}


static void handle_cmd_status(uint8_t reserved){
	prepareTxStatus(&spiTxStatusBuffer, numMsgToSend, reserved);
	spiTxLength = sizeof(SPIHeader);

	if (!SPI_IsReady()) {
			printf("SPI non pronto\n");
	        SPI_reStartMode();
	        return;
	}

	HAL_SPI_TransmitReceive_DMA(
		&hspi1,
		(uint8_t *)&spiTxStatusBuffer,
		(uint8_t *)&spiRxStatusBuffer,
		spiTxLength);

	SPI_ResetWatchdog();
	SPI_SetWatchdog(true);


}


// Se il buffer è pronto, invia i dati
static void handle_cmd_read(void){
	numMsgToSend = 0;

	if (!SPI_IsReady()) {
		//printf("SPI non pronto\n");
		SPI_reStartMode();
		return;
	}
	//printf("SPI TX: %d | SPI RX: %d\n", HAL_SPI_GetState(hspi1), HAL_SPI_GetState(hspi1.hdmarx));
	if ((txBufferReady[0]) && (txBufferTransmit == 0)){
		HAL_SPI_TransmitReceive_DMA(
			&hspi1,
			(uint8_t *)&txPackets[0],
			(uint8_t *)&rxPacket,
			spiTxLength);

		txBufferTransmit = 1;
		txBufferReady[0] = false;


	}else if ((txBufferReady[1]) && (txBufferTransmit == 0)){

		HAL_SPI_TransmitReceive_DMA(
			&hspi1,
			(uint8_t *)&txPackets[1],
			(uint8_t *)&rxPacket,
			spiTxLength);

		txBufferTransmit = 2;
		txBufferReady[1] = false;

	}else{
		// risposta "NOT READY"
		prepareNotReadyPacket(&spiTxStatusBuffer, 0xFF);
		spiTxLength = sizeof(SPIHeader);

		HAL_SPI_TransmitReceive_DMA(
			&hspi1,
			(uint8_t *)&spiTxStatusBuffer,
			(uint8_t *)&spiRxStatusBuffer,
			spiTxLength);

	}

	SPI_ResetWatchdog();
	SPI_SetWatchdog(true);

}

// Se non è un comando conosciuto, invia un errore
static void handle_cmd_default(void){
	SPI_ResetWatchdog();
	SPI_SetWatchdog(true);

	spiTxBuffer[0] = 0xFF;
	spiTxBuffer[1] = 0xFF;
	spiTxBuffer[2] = 0xFF;
	spiTxLength = MESSAGES_DEFAULT;

	if (!SPI_IsReady()) {
		return;
	}

	HAL_SPI_TransmitReceive_DMA(&hspi1, spiTxBuffer, spiRxBuffer, spiTxLength);

}


static void handle_cmd_testMode(void){
	uint8_t test_id   = spiRxBuffer[1];
	spiRxBuffer[1] = 0;

	switch (test_id) {

	  case TEST_SINGLE_VALUE:
		fake_counter = 0;
		ascending = true;

		numMsgToSend = 42;
		handle_cmd_status(test_id);
		numMsgToSend = 42;
		break;

	  case TEST_RAMP:
		//start_time = get_timestamp_us();
		numMsgToSend = fake_counter;
		handle_cmd_status(test_id);

		// Finta logica salita/discesa

		if (ascending) {
			fake_counter++;
			if (fake_counter >= 100) ascending = false;
		} else {
			fake_counter--;
			if (fake_counter == 0) ascending = true;
		}
		//stop_time = get_timestamp_us();
		//uint32_t last_time = stop_time - start_time;
		//printf("Tempo: %lu\n", last_time);
		//printf(">> Entrato in TEST_MODE: id=%d\n", test_id);
		numMsgToSend = 42;
		break;

	  case TEST_FILL_READ_BUFFER:
		 uint8_t test_numMsg = spiRxBuffer[2];

		 handle_cmd_testFillReadBuffer(test_numMsg);
		 break;

	default:
		fake_counter = 0;
		ascending = true;
		handle_cmd_default();
		break;
	}
}


//
static void handle_testFillReadBuffer(uint8_t numMsg) {


	if (numMsg > MAX_MESSAGES_PER_BLOCK) {
	        numMsg = MAX_MESSAGES_PER_BLOCK;
	    }
	spiTxLength = numMsg * sizeof(CAN_Log_Message) + sizeof(SPIHeader);

	if (!SPI_IsReady()) {
		return;
	}

	HAL_SPI_TransmitReceive_DMA(
			&hspi1,
			(uint8_t *)&testTxPacket,
			(uint8_t *)&testRxPacket,
			spiTxLength);

	spiRxBuffer[0] = 0;  // pulizia comando ricevuto
	SPI_ResetWatchdog();
	SPI_SetWatchdog(true);

}


static void handle_cmd_FillBuffer(void){

}


bool SPI_IsPacketValid(SPI_Packet *packet) {
    if (packet->header.marker != SPI_MARKER) return false;

    if (packet->header.messageCount > MAX_MESSAGES_PER_BLOCK)
        return false;

    switch (packet->header.packetType) {
        case CMD_STATUS:
        case CMD_READ:
        case CMD_NOT_READY:
        case CMD_TEST_MODE:
        case CMD_TEST_FILL:
            return true;
        default:
            return false;
    }
}
