/* Copyright (C) 2014-2016 by Will Steelman
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Lib/ScanLib.h>
#include "error.h"
#include "dma.h"

#define UART_Count 2

typedef struct uart_addrs
{
   volatile uint32_t *  rx_pin;
   volatile uint32_t *  tx_pin;
   volatile uint8_t  *  bdh;              // baud rate high
   volatile uint8_t  *  bdl;              // baud rate low
   volatile uint8_t  *  c1;               // control 1
   volatile uint8_t  *  c2;               // control 2
   volatile uint8_t  *  c3;               // control 3
   volatile uint8_t  *  c4;               // control 4
   volatile uint8_t  *  c5;               // control 5
   volatile uint8_t  *  s1;               // status 1
   volatile uint8_t  *  s2;               // status 2
   volatile uint8_t  *  data;             // data
   volatile uint8_t  *  pfifo;            // fifo parameters
   volatile uint8_t  *  cfifo;            // fifo control
   volatile uint8_t  *  sfifo;            // fifo status
   volatile uint8_t  *  tcfifo;           // fifo transmit count
   uint8_t              pin_mux_mode;     // pin mux setting
   uint32_t             clk_gate_mask;    // clock gate mask for SIM_SCGC4
   uint8_t              dma_mux_rx_mask;  // mask for DMA mux
   uint8_t              irq_status_mask;  // mask for IRQ status
} uart_addrs;

typedef struct uart_handle
{
   uint8_t     id;
   uint8_t     dma_rx_chnl;
   uint8_t     tx_fifo_size;
} uart_handle;

typedef const void *uart_handle_t;

#if defined (_mk20dx256vlh7_)
   #define uart0 {                                 \
      .rx_pin           = &PORTA_PCR1,             \
      .tx_pin           = &PORTA_PCR2,             \
      .bdh              = &UART0_BDH,              \
      .bdl              = &UART0_BDL,              \
      .c1               = &UART0_C1,               \
      .c2               = &UART0_C2,               \
      .c3               = &UART0_C3,               \
      .c4               = &UART0_C4,               \
      .c5               = &UART0_C5,               \
      .s1               = &UART0_S1,               \
      .s2               = &UART0_S2,               \
      .data             = &UART0_D,                \
      .pfifo            = &UART0_PFIFO,            \
      .cfifo            = &UART0_CFIFO,            \
      .sfifo            = &UART0_SFIFO,            \
      .sfifo            = &UART0_TCFIFO,           \
      .pin_mux_mode     = 2,                       \
      .clk_gate_mask    = SIM_SCGC4_UART0,         \
      .dma_mux_rx_mask  = DMAMUX_SOURCE_UART0_RX,  \
      .irq_status_mask  = IRQ_UART0_STATUS         \
   }

   #define uart1 {                                 \
      .rx_pin           = &PORTE_PCR0,             \
      .tx_pin           = &PORTE_PCR1,             \
      .bdh              = &UART1_BDH,              \
      .bdl              = &UART1_BDL,              \
      .c1               = &UART1_C1,               \
      .c2               = &UART1_C2,               \
      .c3               = &UART1_C3,               \
      .c4               = &UART1_C4,               \
      .c5               = &UART1_C5,               \
      .s1               = &UART1_S1,               \
      .s2               = &UART1_S2,               \
      .data             = &UART1_D,                \
      .pfifo            = &UART1_PFIFO,            \
      .cfifo            = &UART1_CFIFO,            \
      .sfifo            = &UART1_SFIFO,            \
      .sfifo            = &UART1_TCFIFO,           \
      .pin_mux_mode     = 3,                       \
      .clk_gate_mask    = SIM_SCGC4_UART1,         \
      .dma_mux_rx_mask  = DMAMUX_SOURCE_UART1_RX,  \
      .irq_status_mask  = IRQ_UART1_STATUS         \
   }

#endif

uart_handle_t uart_config (uint8_t uart_id, uint16_t baud);

error_code_t uart_set_baud(uart_handle_t handle, uint16_t baud);

error_code_t uart_rx_dma_setup(uint8_t chnl, uart_handle_t handle, uint32_t *dst, uint32_t buf_size);

uint8_t uart_rx_dma_buffer_position(uart_handle_t handle);

void uart_start(uart_handle_t handle);

void uart_stop(uart_handle_t handle);

uint8_t uart_tx_fifo_size(uart_handle_t handle);

uint8_t uart_tx_fifo_pending(uart_handle_t handle);

void uart_send(uart_handle_t handle, uint8_t data);

