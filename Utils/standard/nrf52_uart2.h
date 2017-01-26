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

#include <stdint.h>
#include <stdbool.h>
#include "error.h"

#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_uarte.h>
#include <core_cm4.h>

#define UART_Count 1

typedef NRF_UARTE_Type uart_dev_t;
static uart_dev_t *uart_devs[1] = {NRF_UARTE0};

typedef struct uart_handle
{
   uint8_t     id;
   uart_dev_t *dev;
   uint8_t    *rx_buf;
   uint8_t     rx_buf_size;
   uint8_t     rx_idx;
   uint8_t     rx_next_idx;
   uint8_t     tx_data;
   uint8_t     tx_pending;
} uart_handle;

typedef const void *uart_handle_t;

uart_handle uart_handles[UART_Count];

#define  DOUBLE_BUF_SIZE 1
#define  UART_PIN_DISCONNECTED 0xFFFFFFFF /**< Value indicating that no pin is connected to this UART register. */
#define  IRQ_PRIORITY      1 

static inline IRQn_Type nrf_drv_get_IRQn(void const * const pinst)                                         
{  
    uint8_t ret = (uint8_t)((uint32_t)pinst>>12U);   
    return (IRQn_Type) ret;                                                                                  
}  

static void uart_irq_enable(IRQn_Type IRQn, uint8_t priority)
{
    NVIC_SetPriority(IRQn, priority);
    NVIC_ClearPendingIRQ(IRQn);
    NVIC_EnableIRQ(IRQn);
}

static uint8_t *next_dma_buffer(uart_handle *h)
{
   h->rx_next_idx = h->rx_idx + DOUBLE_BUF_SIZE;
   if (h->rx_next_idx >= h->rx_buf_size)
      h->rx_next_idx = 0;
   return &h->rx_buf[h->rx_next_idx]; 
}

static inline uart_handle_t uart_config (uint8_t uart_id, uint16_t baud)
{
   if (uart_id >= UART_Count)
      return NULL;

   uart_handle *h = &uart_handles[uart_id];
   uart_dev_t *dev = uart_devs[uart_id];
   h->id = uart_id;
   h->dev = dev;
   h->tx_pending = 0;

   uint8_t tx_pin = 6;
   uint8_t rx_pin = 8;

   nrf_gpio_pin_set(tx_pin);
   nrf_gpio_cfg_output(tx_pin);
   nrf_gpio_cfg_input(rx_pin, NRF_GPIO_PIN_NOPULL);

   nrf_uarte_baudrate_set(dev, NRF_UARTE_BAUDRATE_115200);
   nrf_uarte_configure(dev, 
                       NRF_UARTE_PARITY_EXCLUDED,
                       NRF_UARTE_HWFC_DISABLED);
   nrf_uarte_txrx_pins_set(dev, tx_pin, rx_pin);
 
   nrf_uarte_event_clear(dev, NRF_UARTE_EVENT_ENDRX);
   nrf_uarte_event_clear(dev, NRF_UARTE_EVENT_ENDTX);
   nrf_uarte_event_clear(dev, NRF_UARTE_EVENT_ERROR);
   nrf_uarte_event_clear(dev, NRF_UARTE_EVENT_RXTO);
   nrf_uarte_int_enable(dev, NRF_UARTE_INT_ENDRX_MASK |
                                                 NRF_UARTE_INT_ENDTX_MASK |
                                                 NRF_UARTE_INT_ERROR_MASK |
                                                 NRF_UARTE_INT_RXTO_MASK);
   uart_irq_enable(nrf_drv_get_IRQn((void *)dev), IRQ_PRIORITY);

   nrf_uarte_enable(h->dev); 
//   h->config.use_easy_dma = true;
//   h->config.baudrate = UART_BAUDRATE_BAUDRATE_Baud115200; // neeed to convert input param to nrf format
//   h->config.hwfc = NRF_UART_HWFC_DISABLED;
//   h->config.interrupt_priority = IRQ_PRIORITY;
//   h->config.parity = NRF_UART_PARITY_EXCLUDED;
//   h->config.pselcts = 7;
//   h->config.pselrts = 5;
//   h->config.pselrxd = 8;
//   h->config.pseltxd = 6;
//   h->config.p_context = h;
//
//   nrf_drv_uart_init(&app_uart_inst, &h->config, uart_event_handler);

   return (void*)h;
}

static inline error_code_t uart_set_baud(uart_handle_t handle, uint16_t baud)
{
   uart_handle *h = (uart_handle*)handle;
   nrf_uarte_baudrate_set(h->dev, NRF_UARTE_BAUDRATE_115200);

   return SUCCESS;
}

static inline error_code_t uart_rx_dma_setup(uint8_t chnl, uart_handle_t handle, uint8_t *dst, uint32_t buf_size)
{
   uart_handle *h = (uart_handle*)handle;
   
   h->rx_buf = dst;
   h->rx_buf_size = buf_size;
   h->rx_idx = 0;

   nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_ENDRX);
   nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_RXTO);
   nrf_uarte_rx_buffer_set(h->dev, dst, DOUBLE_BUF_SIZE);
   nrf_uarte_shorts_enable(h->dev, NRF_UARTE_SHORT_ENDRX_STARTRX);

   return SUCCESS;
}

static inline uint8_t uart_rx_dma_buffer_position(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return h->rx_idx; // + nrf_uarte_rx_amount_get(h->dev);
}

static inline void uart_start(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;

   // setup area for next dma transfer
   nrf_uarte_rx_buffer_set(h->dev, next_dma_buffer(h), DOUBLE_BUF_SIZE);
   nrf_uarte_tx_buffer_set(h->dev, &h->tx_data, 1);
   nrf_uarte_task_trigger(h->dev, NRF_UARTE_TASK_STARTRX);
}

static inline void uart_stop(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   nrf_uarte_disable(h->dev); 
}

static inline uint8_t uart_tx_fifo_size(uart_handle_t handle)
{
   return 1;
}

static inline uint8_t uart_tx_fifo_pending(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return h->tx_pending;
}

static inline void uart_send(uart_handle_t handle, uint8_t data)
{
   uart_handle *h = (uart_handle*)handle;
   if (h->tx_pending)
      return;

   h->tx_data = data;
   h->tx_pending = 1;
   
   nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_ENDTX);
   //nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_TXSTOPPED);
   //nrf_uarte_tx_buffer_set(h->dev, &h->tx_data, 1);
   nrf_uarte_task_trigger(h->dev, NRF_UARTE_TASK_STARTTX);
}

