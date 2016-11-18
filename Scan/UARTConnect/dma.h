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

#ifndef DMA_H__
#define DMA_H__

#include <Lib/ScanLib.h>
#include "error.h"

#define DMA_MAX_CHNL_INDEX 15

typedef struct dma_addrs
{
   volatile uint8_t *               mux_config;          // DMAMUX0_CHCFG[n]
   volatile uint16_t *              chnl_csr;            // DMA_TCD[n]_CSR
   volatile uint8_t *               chnl_prio;           // DMA_DCHPRI[n]
   volatile const void * volatile * saddr;               // DMA_TCD[n]_SADDR
   volatile int16_t *               saddr_offset;        // DMA_TCD[n]_SOFF
   volatile void * volatile *       daddr;               // DMA_TCD[n]_DADDR
   volatile int16_t *               daddr_offset;        // DMA_TCD[n]_DOFF
   volatile uint16_t *              attributes;          // DMA_TCD[n]_ATTR
   volatile uint32_t *              byte_cnt;            // DMA_TCD[n]_NBYTES_MLNO
   volatile int32_t *               last_src_adjust;     // DMA_TCD[n]_SLAST
   volatile uint16_t *              citer_loop_count;    // DMA_TCD[n]_CITER_ELINKNO
   volatile uint16_t *              biter_loop_count;    // DMA_TCD[n]_BITER_ELINKNO
   volatile int32_t *               last_dst;            // DMA_TCD[n]_DLASTSGA
   uint32_t                         enable_request_mask; // DMA_ERQ_ERQ[n]
} dma_addrs;

#if defined (_mk20dx256vlh7_)
   #define dma0 {                                      \
      .mux_config          = &DMAMUX0_CHCFG0,          \
      .chnl_prio           = &DMA_DCHPRI0,             \
      .chnl_csr            = &DMA_TCD0_CSR,            \
      .saddr               = &DMA_TCD0_SADDR,          \
      .saddr_offset        = &DMA_TCD0_SOFF,           \
      .attributes          = &DMA_TCD0_ATTR,           \
      .byte_cnt            = &DMA_TCD0_NBYTES_MLNO,    \
      .last_src_adjust     = &DMA_TCD0_SLAST,          \
      .daddr               = &DMA_TCD0_DADDR,          \
      .daddr_offset        = &DMA_TCD0_DOFF,           \
      .citer_loop_count    = &DMA_TCD0_CITER_ELINKNO,  \
      .biter_loop_count    = &DMA_TCD0_BITER_ELINKNO,  \
      .last_dst            = &DMA_TCD0_DLASTSGA,       \
      .enable_request_mask = DMA_ERQ_ERQ0              \
   }

   #define dma1 {                                      \
      .mux_config          = &DMAMUX0_CHCFG1,          \
      .chnl_prio           = &DMA_DCHPRI1,             \
      .chnl_csr            = &DMA_TCD1_CSR,            \
      .saddr               = &DMA_TCD1_SADDR,          \
      .saddr_offset        = &DMA_TCD1_SOFF,           \
      .attributes          = &DMA_TCD1_ATTR,           \
      .byte_cnt            = &DMA_TCD1_NBYTES_MLNO,    \
      .last_src_adjust     = &DMA_TCD1_SLAST,          \
      .daddr               = &DMA_TCD1_DADDR,          \
      .daddr_offset        = &DMA_TCD1_DOFF,           \
      .citer_loop_count    = &DMA_TCD1_CITER_ELINKNO,  \
      .biter_loop_count    = &DMA_TCD1_BITER_ELINKNO,  \
      .last_dst            = &DMA_TCD1_DLASTSGA,       \
      .enable_request_mask = DMA_ERQ_ERQ1              \
   }
   dma_addrs dma_chnls[2] = {dma0, dma1};
#endif

typedef enum
{
   DMA_OFF = 0,
   DMA_ON = 1
} dma_status_t;

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


#endif
