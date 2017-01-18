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

#include "mk20dx_uart.h"

#if defined (_mk20dx256vlh7_)
   uart_addrs uarts[UART_Count] = {uart0, uart1};
#endif

uart_handle uart_handles[UART_Count] = {0};

uart_handle_t uart_config (uint8_t uart_id, uint16_t baud)
{
   if (uart_id >= UART_Count)
      return NULL;

   const uart_addrs *inf = &uarts[uart_id];
   uart_handle *handle = &uart_handles[uart_id];
   handle->id = uart_id;
   handle->dma_rx_chnl = 0xFF;

   // Turn on the clock
   SIM_SCGC4 |= inf->clk_gate_mask; // Disable clock gating

   // pin setup
   *(inf->rx_pin) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
                    PORT_PCR_MUX(inf->pin_mux_mode);
   *(inf->tx_pin) = PORT_PCR_DSE | PORT_PCR_SRE |
                    PORT_PCR_MUX(inf->pin_mux_mode);

   // baud rate
   *(inf->bdh) = (uint8_t)(baud >> 8);
   *(inf->bdl) = (uint8_t)baud;
   *(inf->c4) = 0;

   // 8 bit, Even Parity, Idle Character bit after stop
   // NOTE: For 8 bit with Parity you must enable 9 bit transmission (pg. 1065)
   //       You only need to use UART0_D for 8 bit reading/writing though
   // UART_C1_M UART_C1_PE UART_C1_PT UART_C1_ILT
   *(inf->c1) = UART_C1_M | UART_C1_PE | UART_C1_ILT;

   // Only using Tx Fifos
   *(inf->pfifo) = UART_PFIFO_TXFE;
   handle->tx_fifo_size = (*(inf->pfifo) & UART_PFIFO_TXFIFOSIZE) >> 2;

   return (void*) handle;
}

error_code_t uart_set_baud(uart_handle_t handle, uint16_t baud)
{
   uart_handle *h = (uart_handle*)handle;
   const uart_addrs *inf = &uarts[h->id];

   // baud rate
   *(inf->bdh) = (uint8_t)(baud >> 8);
   *(inf->bdl) = (uint8_t)baud;
   *(inf->c4) = 0;

   return SUCCESS;
}

error_code_t uart_rx_dma_setup(uint8_t chnl, uart_handle_t handle, uint32_t *dst, uint32_t buf_size)
{
   uart_handle *h = (uart_handle*)handle;
   const uart_addrs *inf = &uarts[h->id];

   if (h->dma_rx_chnl != 0xFF)
      return DMA_CHANNEL_IN_USE;
   h->dma_rx_chnl = chnl;

   error_code_t err = dma_config(chnl, inf->dma_mux_rx_mask, (uint32_t*)inf->data, dst, buf_size);
   if (err != DMA_SUCCESS)
      return err;

   // enable DMA requests on receive(requires RX interrupts)
   *inf->c5 = UART_C5_RDMAS;

   // Add interrupts to the vector table
   NVIC_ENABLE_IRQ( inf->irq_status_mask );

   return SUCCESS;
}

uint8_t uart_rx_dma_buffer_position(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return dma_buffer_position(h->dma_rx_chnl);
}

void uart_start(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].c2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
}

void uart_stop(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].c2 = 0;
}

uint8_t uart_tx_fifo_size(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return h->tx_fifo_size;
}

uint8_t uart_tx_fifo_pending(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return *uarts[h->id].tcfifo;
}

void uart_send(uart_handle_t handle, uint8_t data)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].data = data;
}
