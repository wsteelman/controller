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

#include "error.h"

/* Generic ring buffer structure and manipulation functions.  
 * - data is enqueued to the head and dequeued from the tail
 * - buffer is empty when head and tail are equal
 * - buffer is full when head+1 equal tail
 * - note that when the buffer is full, the 'head' slot will be empty
 * - the handle void * is a hack, but the polymorphism modalities in C are limited, 
 *    this at least provides some type erasure via the implicit cast to void * 
 */

typedef struct ring_buffer_t
{
   uint8_t head;
   uint8_t tail;
   uint8_t capacity;
   uint8_t buffer[];
} ring_buffer_t;

typedef void * ring_buffer_handle;

// Reset ring buffer structure
inline void ring_buffer_reset(ring_buffer_handle handle, uint8_t capacity)
{
   ring_buffer_t *buf = (ring_buffer_t*)handle;
   buf->head = 0;
   buf->tail = 0;
   buf->capacity = capacity;
}

// Enqueue a byte to the head pointer
inline error_code_t ring_buffer_enqueue(ring_buffer_handle handle, uint8_t data)
{
   ring_buffer_t *buf = (ring_buffer_t*)handle;
   uint8_t nxt_head = (buf->head + 1) % buf->capacity;
   if (nxt_head == buf->tail)
      return RING_BUFFER_FULL;

   buf->buffer[ buf->head ] = data;
   buf->head = nxt_head;
   return SUCCESS;
}

// Dequeue a byte from the tail pointer
inline error_code_t ring_buffer_dequeue(ring_buffer_handle handle, uint8_t *data)
{
   ring_buffer_t *buf = (ring_buffer_t*)handle;
   if (buf->tail == buf->head)
      return RING_BUFFER_EMPTY;

   *data = buf->buffer[ buf->tail ];
   buf->tail = (buf->tail + 1) % buf->capacity;
   return SUCCESS;
}

// Compute the usage of the buffer based on the pointer values, compensating for wrap around
inline uint8_t ring_buffer_size(const ring_buffer_handle handle)
{
   const ring_buffer_t *buf = (const ring_buffer_t*)handle;
   uint8_t size = buf->head - buf->tail;
   if (buf->head < buf->tail)
      size += buf->capacity;
   return size;
}

// If head+1 == tail, buffer is full
inline uint8_t ring_buffer_full(const ring_buffer_handle handle)
{
   const ring_buffer_t *buf = (const ring_buffer_t*)handle;
   uint8_t nxt_head = (buf->head + 1) % buf->capacity;
   return nxt_head == buf->tail;
}

// If head == tail, buffer is empty
inline uint8_t ring_buffer_empty(const ring_buffer_handle handle)
{
   const ring_buffer_t *buf = (const ring_buffer_t*)handle;
   return buf->tail == buf->head;
}

