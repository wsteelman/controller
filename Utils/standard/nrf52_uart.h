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
#include "sdk_config.h"

#include <nrf_drv_uart.h>
#include <nrf_assert.h>
#include <sdk_common.h>

#define UART_Count 2

typedef struct uart_handle
{
   uint8_t     id;
   uint8_t    *rx_buf;
   uint8_t     rx_buf_size;
   uint8_t     rx_idx;
   uint8_t     tx_data;
   uint8_t     tx_pending;
   nrf_drv_uart_config_t config;
} uart_handle;

typedef const void *uart_handle_t;

static uart_handle uart_handles[UART_Count] = {0};

#define  UART_PIN_DISCONNECTED 0xFFFFFFFF /**< Value indicating that no pin is connected to this UART register. */
#define IRQ_PRIORITY       6
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);

static inline void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
   uart_handle *h = (uart_handle*)p_context;
   if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
   {
      //app_uart_evt_t app_uart_event;
      //app_uart_event.evt_type   = APP_UART_DATA;
      //app_uart_event.data.value = p_event->data.rxtx.p_data[0];
      (void)nrf_drv_uart_rx(&app_uart_inst, h->rx_buf, 1);
      h->rx_idx = (h->rx_idx + 1) % h->rx_buf_size;
   }
   else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
   {
      //app_uart_evt_t app_uart_event;
      //app_uart_event.evt_type                 = APP_UART_COMMUNICATION_ERROR;
      //app_uart_event.data.error_communication = p_event->data.error.error_mask;
      //(void)nrf_drv_uart_rx(&app_uart_inst, rx_buffer, 1);
      //m_event_handler(&app_uart_event);
   }
   else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
   {
      h->tx_pending = false;
      // Last byte from FIFO transmitted, notify the application.
      // Notify that new data is available if this was first byte put in the buffer.
      //app_uart_evt_t app_uart_event;
      //app_uart_event.evt_type = APP_UART_TX_EMPTY;
      //m_event_handler(&app_uart_event);
   }
}


static inline uart_handle_t uart_config (uint8_t uart_id, uint16_t baud)
{
   if (uart_id >= UART_Count)
      return NULL;

   uart_handle *h = &uart_handles[uart_id];
   h->id = uart_id;
   h->tx_pending = false;

   h->config.use_easy_dma = true;
   h->config.baudrate = UART_BAUDRATE_BAUDRATE_Baud115200; // neeed to convert input param to nrf format
   h->config.hwfc = NRF_UART_HWFC_DISABLED;
   h->config.interrupt_priority = IRQ_PRIORITY;
   h->config.parity = NRF_UART_PARITY_EXCLUDED;
   h->config.pselcts = 7;
   h->config.pselrts = 5;
   h->config.pselrxd = 8;
   h->config.pseltxd = 6;
   h->config.p_context = h;

   nrf_drv_uart_init(&app_uart_inst, &h->config, uart_event_handler);
   //VERIFY_SUCCESS(err_code);

   return (void*)h;
}

static inline error_code_t uart_set_baud(uart_handle_t handle, uint16_t baud)
{
//   uart_handle *h = (uart_handle*)handle;
//   const uart_addrs *inf = &uarts[h->id];
//
//   // baud rate
//   *(inf->bdh) = (uint8_t)(baud >> 8);
//   *(inf->bdl) = (uint8_t)baud;
//   *(inf->c4) = 0;

   return SUCCESS;
}

static inline error_code_t uart_rx_dma_setup(uint8_t chnl, uart_handle_t handle, uint8_t *dst, uint32_t buf_size)
{
   uart_handle *h = (uart_handle*)handle;

   h->rx_buf = dst;
   h->rx_buf_size = buf_size;
   h->rx_idx = 0;
   nrf_drv_uart_rx(&app_uart_inst, h->rx_buf, 1);

   return SUCCESS;
}

static inline uint8_t uart_rx_dma_buffer_position(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return h->rx_idx;
}

static inline void uart_start(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   // Turn on receiver if RX pin is connected
   if (h->config.pselrxd != UART_PIN_DISCONNECTED)
   {
#ifdef UARTE_PRESENT
      if (!h->config.use_easy_dma)
#endif
      {
         nrf_drv_uart_rx_enable(&app_uart_inst);
      }
   }
}

static inline void uart_stop(uart_handle_t handle)
{
   nrf_drv_uart_rx_enable(&app_uart_inst);
}

static inline uint8_t uart_tx_fifo_size(uart_handle_t handle)
{
   //uart_handle *h = (uart_handle*)handle;
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
   h->tx_data = data;
   h->tx_pending = true;
   nrf_drv_uart_tx(&app_uart_inst, &h->tx_data, 1);
   
   //ret_code_t ret =  nrf_drv_uart_tx(&app_uart_inst, &h->tx_data, 1);
   //if (NRF_ERROR_BUSY == ret)
   //{
   //    return NRF_ERROR_NO_MEM;
   //}
   //else if (ret != NRF_SUCCESS)
   //{
   //    return NRF_ERROR_INTERNAL;
   //}
   //else
   //{
   //    return NRF_SUCCESS;
   //}
}

