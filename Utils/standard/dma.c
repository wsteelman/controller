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

#include "dma.h"

#if defined (_mk20dx256vlh7_)
   dma_addrs dma_chnls[2] = {dma0, dma1};
#endif

// ----- Variables -----

dma_status_t dma_global_status = DMA_OFF;
dma_status_t dma_chnl_status[DMA_MAX_CHNL_INDEX+1] = {0};

// ----- Functions -----

uint8_t dma_buffer_position(uint8_t chnl_id)
{
   return *dma_chnls[chnl_id].citer_loop_count;
}

error_code_t dma_config (uint8_t chnl_id, uint8_t src_mux_id, uint32_t *src, uint32_t *dst, uint32_t buf_size)
{
   if (dma_global_status == DMA_OFF)
   {
      // clear DMA control
      DMA_CR = 0;

      // Setup DMA clocks
      SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
      SIM_SCGC7 |= SIM_SCGC7_DMA;
      dma_global_status = DMA_ON;
   }

   if (chnl_id > DMA_MAX_CHNL_INDEX)
      return DMA_INVALID_CHANNEL_ID;

   if (dma_chnl_status[chnl_id] == DMA_ON)
      return DMA_CHANNEL_IN_USE;
   dma_chnl_status[chnl_id] = DMA_ON;

   dma_addrs *chnl = &dma_chnls[chnl_id];

   // disable channel
   *chnl->mux_config = 0;

   // clear csr
   *chnl->chnl_csr = 0;

   // set priority
   *chnl->chnl_prio = chnl_id;

   //// clear interrupts
   DMA_EEI = 0;

   // setup TCD
   *chnl->saddr = src;
   *chnl->saddr_offset = 0;
   *chnl->last_src_adjust = 0;

   // no modulo, 8-bit transfer dize
   *chnl->attributes = DMA_TCD_ATTR_SMOD(0) | DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DMOD(0) | DMA_TCD_ATTR_DSIZE(0);

   // One bytes transferred at a time
   *chnl->byte_cnt = 1;

   // destination
   *chnl->daddr = dst;
   *chnl->daddr_offset = 1;

   // Incoming byte, increment by 1 in the rx buffer
   *chnl->daddr_offset = 1;

   // Single major loop, must be the same value
   *chnl->citer_loop_count = buf_size;
   *chnl->biter_loop_count = buf_size;

   // reset buffer when full
   *chnl->last_dst = -(buf_size);

   // enable dma channel
   DMA_ERQ |= chnl->enable_request_mask;

   // setup DMA routing
   *chnl->mux_config = DMAMUX_ENABLE | src_mux_id;

   return DMA_SUCCESS;
}


