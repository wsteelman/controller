#pragma once

#include "error.h"
#include "msg.h"

#define UART_Buffer_Size UARTConnectBufSize_define

// UART Rx/Tx Status
typedef enum UARTStatus {
   UARTStatus_Wait    = 0, // Waiting  Rx: for SYN  Tx: for current command copy to finish
   UARTStatus_SYN     = 1, // Rx: SYN Received, waiting for SOH
   UARTStatus_SOH     = 2, // Rx: SOH Received, waiting for Command
   UARTStatus_Command = 3, // Rx: Command Received, waiting for data
   UARTStatus_Data    = 4, // Rx: Command Received, waiting for data
   UARTStatus_Ready   = 5, // Tx: Ready to send commands
} UARTStatus;

typedef struct uart_ring_buf_t {
   uint8_t head;
   uint8_t tail;
   uint8_t capacity;
   uint8_t buffer[UART_Buffer_Size];
} uart_ring_buf_t;

typedef struct uart_dma_buf_t {
   uint8_t  buffer[UART_Buffer_Size];
   uint16_t last_read;
} uart_dma_buf_t;

typedef struct uart_status_tx_t {
   UARTStatus status;
   uint8_t    lock;
} uart_status_tx_t;

typedef uint8_t (*rx_callback_t)(const msg_header*);
typedef uint8_t (*rx_byte_callback_t)(uint8_t byte, uint8_t uart_num, uint8_t cmd, uint16_t *bytes);

typedef struct uart_message_pipe_t
{
   uint8_t              configured;
   uint8_t              id;
   uart_handle_t        uart_handle;
   uart_status_tx_t     tx_status;
   uart_ring_buf_t      tx_buf;
   UARTStatus           rx_status;
   uart_dma_buf_t       rx_buf;
   uint8_t              rx_msg[256];
   uint8_t              rx_msg_index;
   rx_callback_t        rx_callbacks[CmdCommand_TOP];
} uart_message_pipe_t;

void upipe_enable_debug(uint8_t debug);

error_code_t upipe_reset(uart_message_pipe_t *pipe, uint8_t capcity);

error_code_t upipe_init(uart_message_pipe_t *pipe, uint8_t uart_id,
                        uint8_t baud, uint8_t buffer_size);

error_code_t upipe_send_msg(uart_message_pipe_t *pipe, const msg_header *hdr);

error_code_t upipe_send_variable_msg(uart_message_pipe_t *pipe, msg_header *hdr, const uint8_t *var, uint8_t var_size);

error_code_t upipe_process(uart_message_pipe_t *pipe);

error_code_t upipe_send_idle(uart_message_pipe_t *pipe, uint8_t len);

void upipe_register_callback(uart_message_pipe_t *pipe, command cmd, rx_callback_t func);

// temp functions for transition
error_code_t upipe_send_bytes(uart_message_pipe_t *pipe, const uint8_t *buffer, uint8_t count);
