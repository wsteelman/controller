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

typedef enum command {
   CmdCableCheck,       // Comm check

   CmdIdRequest,        // Slave initialization (request id from master)
   CmdIdEnumeration,    // Slave initialization (begin enumeration from master)
   CmdIdReport,         // Slave initialization complete, report id to master

   CmdScanCode,         // ScanCode event status change
   CmdAnimation,        // Master trigger animation event (same command is sent back to master when ready)

   CmdRemoteCapability, // Activate a capability on the given node
   CmdRemoteOutput,     // Remote debug output from a given node
   CmdRemoteInput,      // Remote command to send to a given node's debug cli

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


