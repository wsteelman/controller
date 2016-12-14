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

// ----- Includes -----

// Compiler Includes
#include <Lib/ScanLib.h>

// Project Includes
#include <kll_defs.h>
#include <print.h>

// Local Includes
#include "uart.h"
#include "ring_buffer.h"
#include "uart_message_pipe.h"

// -- Control Variables --
uint8_t upipe_debug = 0;      // Set 1 for debug
uart_message_pipe_t pipes[UART_Count] = {0};

// ----- Internal Functions -----

void upipe_tx_process(uart_message_pipe_t *pipe)
{
   uart_ring_buf_t *rbuf = &pipe->tx_buf;
   uint8_t tx_pending = uart_tx_fifo_pending(pipe->uart_handle);
   if ( ring_buffer_empty(rbuf) || tx_pending > 0 )
      return;

   uint8_t fifoSize = uart_tx_fifo_size(pipe->uart_handle);
   if ( fifoSize == 0 )
      fifoSize = 1;
   if ( upipe_debug )
   {
      print( "TxFIFO " );
      printHex( pipe->id );
      print(" - " );
      printHex( fifoSize );
      print("/");
      printHex( uart_tx_fifo_pending(pipe->uart_handle) );
      print("/");
      printHex( ring_buffer_size(rbuf) );
      print( NL );
   }

   uint8_t data = 0;
   fifoSize -= tx_pending;
   while ( fifoSize-- != 0 )
   {
      error_code_t err = ring_buffer_dequeue(rbuf, &data);
      if ( err != SUCCESS )
         break;
      uart_send(pipe->uart_handle, data);
   }
}

void upipe_rx_process(uart_message_pipe_t *pipe)
{
   // Determine current position to read until
   uint16_t bufpos = uart_rx_dma_buffer_position(pipe->uart_handle);

   // Process each of the new bytes
   // Even if we receive more bytes during processing, wait until the next check so we don't starve other tasks
   while ( bufpos != pipe->rx_buf.last_read )
   {
      // If the last_read byte is at the buffer edge, roll back to beginning
      if ( pipe->rx_buf.last_read == 0 )
      {
         pipe->rx_buf.last_read = UART_Buffer_Size;

         // Check to see if we're at the boundary
         if ( bufpos == UART_Buffer_Size )
            break;
      }

      // Read the byte out of Rx DMA buffer
      uint8_t byte = pipe->rx_buf.buffer[ UART_Buffer_Size - pipe->rx_buf.last_read-- ];
      pipe->rx_msg[pipe->rx_msg_index++] = byte;

      if ( upipe_debug )
      {
         printHex( byte );
         print(" ");
      }

      // Process UART byte
      switch ( pipe->rx_status )
      {
      // Every packet must start with a SYN / 0x16
      case UARTStatus_Wait:
         if ( upipe_debug )
         {
            print(" Wait ");
         }
         pipe->rx_status = byte == 0x16 ? UARTStatus_SYN : UARTStatus_Wait;
         pipe->rx_msg_index = 1;
         pipe->rx_msg[0] = byte;
         break;

      // After a SYN, there must be a SOH / 0x01
      case UARTStatus_SYN:
         if ( upipe_debug )
         {
            print(" SYN ");
         }
         pipe->rx_status = byte == 0x01 ? UARTStatus_SOH : UARTStatus_Wait;
         break;

      // After a SOH the packet structure may diverge a bit
      // This is the packet type field (refer to the Command enum)
      // For very small packets (e.g. IdRequest) this is all that's required to take action
      case UARTStatus_SOH:
      {
         if ( upipe_debug )
         {
            print(" SOH ");
         }
         pipe->rx_status = byte > 0x00 ? UARTStatus_Data : UARTStatus_Wait;
         break;
      }

      // After the packet type has been deciphered do Command specific processing
      // Until the Command has received all the bytes it requires the UART buffer stays in this state
      case UARTStatus_Data:
      {
         if ( upipe_debug )
         {
            print(" DATA ");
         }
         pipe->rx_status = UARTStatus_Command;
         break;
      }

      case UARTStatus_Command:
      {
         msg_header *hdr = (msg_header*)pipe->rx_msg;
         if (pipe->rx_msg_index >= hdr->bytes)
         {
            /* Call specific UARTConnect command receive function */
            rx_callback_list_t *cb_list = &pipe->rx_callbacks[hdr->cmd];
            for (uint8_t i = 0; i < cb_list->count; ++i)
            {
               cb_list->callbacks[i](hdr);
            }
            pipe->rx_status = UARTStatus_Wait;
            pipe->rx_msg_index = 0;
         }
         break;
      }

      //// Unknown status, should never get here
      default:
         erro_msg("Invalid UARTStatus...");
         pipe->rx_status = UARTStatus_Wait;
         pipe->rx_msg_index = 0;
         continue;
      }

      if ( upipe_debug )
      {
         print( NL );
      }
   }

}

void uart_lock_tx(uart_status_tx_t *status)
{
   // TODO This code does not actually guarantee mutual exclusion, need to get better mutex
   // ARM synchronization primitives: http://infocenter.arm.com/help/topic/com.arm.doc.dht0008a/DHT0008A_arm_synchronization_primitives.pdf

   /* First, secure place in line for the resource */
   while ( status->lock );
   status->lock = 1;
   /* Next, wait unit the UART is ready */
   while ( status->status != UARTStatus_Ready );
   status->status = UARTStatus_Wait;
}

void uart_unlock_tx(uart_status_tx_t *status)
{
   /* Ready the UART */
   status->status = UARTStatus_Ready;
   /* Unlock the resource */
   status->lock = 0;
}

error_code_t upipe_send_bytes(uart_message_pipe_t *pipe, const uint8_t *buffer, uint8_t count)
{
   // Too big to fit into buffer
   if ( count > pipe->tx_buf.capacity )
   {
      erro_msg("Too big of a command to fit into the buffer...");
      return RING_BUFFER_FULL;
   }

   uart_ring_buf_t *buf = &pipe->tx_buf;
   for (uint8_t i = 0; i < count; i++)
   {
      while (ring_buffer_enqueue(buf, *buffer) != SUCCESS)
      {
         delay( 1 );
      }
      if ( upipe_debug )
      {
         printHex( *buffer );
         print( NL );
      }
      buffer++;
   }
   return SUCCESS;
}

// ---- API Functions -----

void upipe_enable_debug(uint8_t debug)
{
   upipe_debug = debug;
}

error_code_t upipe_reset(uart_message_pipe_t *pipe, uint8_t capacity)
{
   ring_buffer_reset(&pipe->tx_buf, capacity);
   memset( (void*)&pipe->tx_status, 0, sizeof( uart_status_tx_t ) );
   pipe->tx_status.status = UARTStatus_Ready;
   pipe->rx_buf.last_read = UART_Buffer_Size;
   pipe->rx_status = UARTStatus_Wait;
   return SUCCESS;
}

error_code_t upipe_init(uart_message_pipe_t **return_pipe, uint8_t uart_id, uint8_t baud, uint8_t buffer_size)
{
   uart_message_pipe_t *pipe = &pipes[uart_id];
   if (pipe->configured)
      return uart_set_baud(pipe->uart_handle, baud);

   memset(pipe, 0x00, sizeof(uart_message_pipe_t));
   error_code_t err = SUCCESS;
   pipe->configured = 0;
   pipe->id = uart_id;

   // configure the UART
   pipe->uart_handle = uart_config(uart_id, baud);
   if (pipe->uart_handle == NULL)
      return FAILURE;

   // attach UART rx to dma buffer
   err = uart_rx_dma_setup(uart_id, pipe->uart_handle, (uint32_t*)pipe->rx_buf.buffer, buffer_size);
   if (err != SUCCESS)
      return err;

   // reset all the buffers
   err = upipe_reset(pipe, buffer_size);
   if (err != SUCCESS)
      return err;

   // enable the UART
   uart_start(pipe->uart_handle);

   pipe->configured = 1;
   *return_pipe = pipe;
   return err;
}

error_code_t upipe_send_msg(uart_message_pipe_t *pipe, const msg_header *hdr)
{
   error_code_t err = SUCCESS;
   uart_lock_tx(&pipe->tx_status);
   err = upipe_send_bytes(pipe, (const uint8_t*)hdr, hdr->bytes);
   // unlock on success or failure
   uart_unlock_tx(&pipe->tx_status);
   return err;
}

error_code_t upipe_send_variable_msg(uart_message_pipe_t *pipe, msg_header *hdr, const uint8_t *var, uint8_t var_size)
{
   error_code_t err = SUCCESS;
   uart_lock_tx(&pipe->tx_status);

   uint8_t hdr_size = hdr->bytes;
   hdr->bytes += var_size;
   err = upipe_send_bytes(pipe, (uint8_t*)hdr, hdr_size);
   if (err != SUCCESS)
   {
      // reset bytes in header so the above hdr->bytes += works 
      hdr->bytes = hdr_size;
      uart_unlock_tx(&pipe->tx_status);
      return err;
   }
   err = upipe_send_bytes(pipe, var, var_size);
   // reset bytes in header so the above hdr->bytes += works 
   hdr->bytes = hdr_size;
   // unlock on success or failure
   uart_unlock_tx(&pipe->tx_status);
   return err;

}

error_code_t upipe_send_idle(uart_message_pipe_t *pipe, uint8_t len)
{
   uart_lock_tx(&pipe->tx_status);

   uint8_t value = CmdCommand_SYN;
   for (uint8_t i = 0; i < len; ++i)
   {
      upipe_send_bytes(pipe, &value, 1);
   }

   uart_unlock_tx(&pipe->tx_status);
   return SUCCESS;
}

void upipe_register_callback(uart_message_pipe_t *pipe, command cmd, rx_callback_t func)
{
   if (pipe->rx_callbacks[cmd].count >= MAX_Subscribers)
      return;

   pipe->rx_callbacks[cmd].callbacks[pipe->rx_callbacks[cmd].count] = func;
   pipe->rx_callbacks[cmd].count++;
}

error_code_t upipe_process(uart_message_pipe_t *pipe)
{
   upipe_tx_process(pipe);
   upipe_rx_process(pipe);
   return SUCCESS;
}
