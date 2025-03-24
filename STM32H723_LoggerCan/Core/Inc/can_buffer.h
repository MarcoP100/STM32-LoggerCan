#ifndef CAN_BUFFER_H
#define CAN_BUFFER_H

#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timer_utils.h"


#define CAN_BUFFER_SIZE 100  // Numero massimo di messaggi salvati

#define CAN_ID_START 0x400
#define CAN_ID_END   0x409
#define TIMEOUT_MAX  1500  // Massimo tempo accettato tra due messaggi (us * 0)
#define TIMEOUT_MIN  500 // Minimo tempo accettato (ms)
#define REPORT_INTERVAL 10000 // Intervallo report in ms

extern volatile uint32_t last_timestamps[10];  // Ultimi tempi di ricezione
extern volatile uint16_t anomaly_counts[10];   // Contatore anomalie
extern volatile uint32_t msg_counter[10];
extern volatile uint32_t min_delta[10];
extern volatile uint32_t max_delta[10];
extern volatile uint32_t ok_timestamp;
extern volatile uint32_t ok_timestamp_read;

extern volatile uint32_t durata_interrupt;

extern volatile uint32_t num_fifo0;
extern volatile uint32_t num_fifo1;


typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t flags;
    uint32_t timestamp;
    uint32_t counter;
} CAN_Message;


// buffer
#define BLOCK_SIZE 64
#define BUFFER_COUNT 3
typedef struct {
    CAN_Message messages[BLOCK_SIZE];
    uint16_t index;
    bool full;
    uint32_t lastMessageTimestamp;  // per timeout (in µs)
} CANBuffer;

extern CANBuffer canBuffers[BUFFER_COUNT];
extern volatile uint8_t activeWriteBuffer = 0;


void buffer_write(CAN_Message *msg);
int buffer_read(CAN_Message *msg);
int buffer_has_data(void);
void test_can_transmit(FDCAN_HandleTypeDef *hfdcan);
void test_can_loopback(FDCAN_HandleTypeDef *hfdcan);
void check_can_rx_polling(FDCAN_HandleTypeDef *hfdcan);
void check_msg_timestamp(CAN_Message *msg);
uint8_t FDCAN_GetRxDataLength(uint32_t dlc);
void process_fifo_rx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifoIndex);

#endif // CAN_BUFFER_H
