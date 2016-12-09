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

typedef enum
{
   FAILURE                 = 0x00,
   SUCCESS                 = 0x01,
   DMA_SUCCESS             = 0x02,
   DMA_INVALID_CHANNEL_ID  = 0x03,
   DMA_CHANNEL_IN_USE      = 0x04,
   RING_BUFFER_FULL        = 0x05,
   RING_BUFFER_EMPTY       = 0x06,
} error_code_t;

