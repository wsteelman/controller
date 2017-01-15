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

#include <kll_defs.h>

typedef enum command {
   CmdCableCheck,       // Comm check

   CmdIdRequest,              // Slave initialization (request id from master)
   CmdIdEnumeration,          // Slave initialization (begin enumeration from master)
   CmdIdReport,               // Slave initialization complete, report id to master

   CmdScanCode,               // ScanCode event status change
   CmdAnimation,              // Master trigger animation event (same command is sent back to master when ready)

   CmdRemoteCapability,       // Activate a capability on the given node
   CmdRemoteOutput,           // Remote debug output from a given node
   CmdRemoteInput,            // Remote command to send to a given node's debug cli
   CmdEnableMaster,           // Enable master on secondary node

   CmdCommand_TOP,      // Enum bounds
   CmdCommand_SYN = 0x16, // Reserved for error handling
} command;

typedef struct msg_header
{
   uint8_t syn;
   uint8_t soh;
   uint8_t bytes;
   command cmd;
} msg_header;

typedef struct cable_check_msg_t
{
   msg_header     header;
   uint8_t        len;
   uint8_t        pattern[UARTConnectCableCheckLength_define];
} cable_check_msg_t;
typedef struct id_request_msg_t
{
   msg_header     header;
   uint8_t        payload;
} id_request_msg_t;

typedef struct id_enum_msg_t
{
   msg_header     header;
   uint8_t        id;
} id_enum_msg_t;

typedef struct id_report_msg_t
{
   msg_header     header;
   uint8_t        id;
} id_report_msg_t;

typedef struct remote_capability_msg_t
{
   msg_header     header;
   uint8_t        id;
   uint8_t        capability_index;
   uint8_t        state;
   uint8_t        state_type;
   uint8_t        num_args;
   uint8_t        args[];
} remote_capability_msg_t;

typedef enum protocol_t
{
   usb_protocol,
   uart_protocol,
   protocol_count,
} protocol_t;

typedef struct enable_master_msg_t
{
   msg_header     header;
   uint8_t        id;
   protocol_t     output_protocol;
} enable_master_msg_t;

