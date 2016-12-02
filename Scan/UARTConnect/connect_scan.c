/* Copyright (C) 2014-2016 by Jacob Alexander
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
#include <cli.h>
#include <kll_defs.h>
#include <led.h>
#include <print.h>
#include <macro.h>

// Local Includes
#include "connect_scan.h"
#include "uart.h"
#include "ring_buffer.h"
#include "uart_message_pipe.h"

// ----- Defines -----

#define UART_Num_Interfaces 2
#define UART_Master 1
#define UART_Slave  0
#define UART_Buffer_Size UARTConnectBufSize_define


// ----- Function Declarations -----

// CLI Functions
void cliFunc_connectCmd ( char *args );
void cliFunc_connectDbg ( char *args );
void cliFunc_connectIdl ( char *args );
void cliFunc_connectLst ( char *args );
void cliFunc_connectMst ( char *args );
void cliFunc_connectRst ( char *args );
void cliFunc_connectSts ( char *args );


// ----- Structs -----

typedef struct UARTDMABuf {
   uint8_t  buffer[UART_Buffer_Size];
   uint16_t last_read;
} UARTDMABuf;

typedef struct UARTStatusRx {
   UARTStatus status;
   Command    command;
   uint16_t   bytes_waiting;
} UARTStatusRx;

typedef struct UARTStatusTx {
   UARTStatus status;
   uint8_t    lock;
} UARTStatusTx;

// ----- Messages -----

typedef struct simple_msg_t
{
   msg_header     header;
} simple_msg_t;

typedef struct cable_check_msg_t
{
   msg_header     header;
   uint8_t        len;
   uint8_t        pattern[UARTConnectCableCheckLength_define];
} cable_check_msg_t;

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

typedef struct scan_code_msg_t
{
   msg_header     header;
   uint8_t        id;
   uint8_t        count;
   uint8_t        codes[];
} scan_code_msg_t;

typedef struct animation_msg_t
{
   msg_header     header;
   uint8_t        id;
   uint8_t        param_count;
   uint8_t        params[];
} animation_msg_t;

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

cable_check_msg_t       cable_check_msg         = { {CmdCommand_SYN, 0x01, sizeof(cable_check_msg_t), CmdCableCheck}, UARTConnectCableCheckLength_define, {0xD2}};
simple_msg_t            id_request_msg          = { {CmdCommand_SYN, 0x01, sizeof(simple_msg_t), CmdIdRequest} };
id_enum_msg_t           id_enum_msg             = { {CmdCommand_SYN, 0x01, sizeof(id_enum_msg_t), CmdIdEnumeration}, 0xFF};
id_report_msg_t         id_report_msg           = { {CmdCommand_SYN, 0x01, sizeof(id_report_msg_t), CmdIdReport}, 0xFF};
scan_code_msg_t         scan_code_msg           = { {CmdCommand_SYN, 0x01, sizeof(scan_code_msg_t), CmdScanCode}, 0xFF, 0};
animation_msg_t         animation_msg           = { {CmdCommand_SYN, 0x01, sizeof(animation_msg_t), CmdAnimation}, 0xFF, 0};
remote_capability_msg_t remote_capability_msg   = { {CmdCommand_SYN, 0x01, sizeof(remote_capability_msg_t), CmdRemoteCapability}, 0xFF, 0, 0, 0, 0};


// ----- Variables -----

// Connect Module command dictionary
CLIDict_Entry( connectCmd,  "Sends a command via UART Connect, first arg is which uart, next arg is the command, rest are the arguments." );
CLIDict_Entry( connectDbg,  "Toggle UARTConnect debug mode." );
CLIDict_Entry( connectIdl,  "Sends N number of Idle commands, 2 is the default value, and should be sufficient in most cases." );
CLIDict_Entry( connectLst,  "Lists available UARTConnect commands and index id" );
CLIDict_Entry( connectMst,  "Sets the device as master. Use argument of s to set as slave." );
CLIDict_Entry( connectRst,  "Resets both Rx and Tx connect buffers and state variables." );
CLIDict_Entry( connectSts,  "UARTConnect status." );
CLIDict_Def( uartConnectCLIDict, "UARTConnect Module Commands" ) = {
   CLIDict_Item( connectCmd ),
   CLIDict_Item( connectDbg ),
   CLIDict_Item( connectIdl ),
   CLIDict_Item( connectLst ),
   CLIDict_Item( connectMst ),
   CLIDict_Item( connectRst ),
   CLIDict_Item( connectSts ),
   { 0, 0, 0 } // Null entry for dictionary end
};


// -- Connect Device Id Variables --
uint8_t Connect_id = 255; // Invalid, unset
uint8_t Connect_master = 0;
uint8_t Connect_maxId = 0;


// -- Control Variables --
uint32_t Connect_lastCheck = 0; // Cable Check scheduler
uint8_t Connect_debug = 0;      // Set 1 for debug
uint8_t Connect_override = 0;   // Prevents master from automatically being set

volatile uint8_t uarts_configured = 0;


// -- Rx Variables --

uart_dma_buf_t *      uart_rx_buf[UART_Num_Interfaces];
volatile UARTStatusRx uart_rx_status[UART_Num_Interfaces];


// -- Tx Variables --

UARTStatusTx uart_tx_status[UART_Num_Interfaces];

// -- UART Variables --
uart_handle_t slave_handle = NULL;
uart_handle_t master_handle = NULL;
uart_message_pipe_t slave_pipe;
uart_message_pipe_t master_pipe;


// -- Ring Buffer Convenience Functions --
void uart_lockTx( uint8_t uartNum )
{
   // TODO This code does not actually guarantee mutual exclusion, need to get better mutex
   // ARM synchronization primitives: http://infocenter.arm.com/help/topic/com.arm.doc.dht0008a/DHT0008A_arm_synchronization_primitives.pdf

   /* First, secure place in line for the resource */
   while ( uart_tx_status[ uartNum ].lock );
   uart_tx_status[ uartNum ].lock = 1;
   /* Next, wait unit the UART is ready */
   while ( uart_tx_status[ uartNum ].status != UARTStatus_Ready );
   uart_tx_status[ uartNum ].status = UARTStatus_Wait;
}

void uart_unlockTx( uint8_t uartNum )
{
   /* Ready the UART */
   uart_tx_status[ uartNum ].status = UARTStatus_Ready;
   /* Unlock the resource */
   uart_tx_status[ uartNum ].lock = 0;
}

void Connect_addBytes( uint8_t *buffer, uint8_t count, uint8_t uart )
{
   if (uart == UART_Slave)
      upipe_send_bytes(&slave_pipe, buffer, count);
   else if (uart == UART_Master)
      upipe_send_bytes(&master_pipe, buffer, count);
}


// -- Connect send functions --

// patternLen defines how many bytes should the incrementing pattern have
void Connect_send_CableCheck()
{
   // Send 0xD2 (11010010) for each argument
   uint8_t value = 0xD2;
   for ( uint8_t c = 0; c < UARTConnectCableCheckLength_define; c++ )
   {
      cable_check_msg.pattern[c] = value;
   }
   upipe_send_msg(&master_pipe, &cable_check_msg.header);
   upipe_send_msg(&slave_pipe, &cable_check_msg.header);
}

void Connect_send_IdRequest()
{
   upipe_send_msg(&master_pipe, &id_request_msg.header);
}

// id is the value the next slave should enumerate as
void Connect_send_IdEnumeration( uint8_t id )
{
   id_enum_msg.id = id;
   upipe_send_msg(&slave_pipe, &id_enum_msg.header);
}

// id is the currently assigned id to the slave
void Connect_send_IdReport( uint8_t id )
{
   id_report_msg.id = id;
   upipe_send_msg(&master_pipe, &id_report_msg.header);
}

// id is the currently assigned id to the slave
// scanCodeStateList is an array of [scancode, state]'s (8 bit values)
// numScanCodes is the number of scan codes to parse from array
void Connect_send_ScanCode( uint8_t id, TriggerGuide *scanCodeStateList, uint8_t numScanCodes )
{
   // Prepare header
   scan_code_msg.id = id;
   scan_code_msg.count = numScanCodes;

   // compute variable byte count of scan code list
   uint8_t code_bytes = numScanCodes * TriggerGuideSize;

   // Send message
   upipe_send_variable_msg(&master_pipe, &scan_code_msg.header, (uint8_t*)scanCodeStateList, code_bytes);
}

// id is the currently assigned id to the slave
// paramList is an array of [param, value]'s (8 bit values)
// numParams is the number of params to parse from the array
void Connect_send_Animation( uint8_t id, uint8_t *paramList, uint8_t numParams )
{
   // Prepare header
   animation_msg.id = id;
   animation_msg.param_count = numParams;

   // Send message
   upipe_send_variable_msg(&slave_pipe, &animation_msg.header, paramList, numParams);
}

// Send a remote capability command using capability index
// This may not be what's expected (especially if the firmware is not the same on each node)
// To broadcast to all slave nodes, set id to 255 instead of a specific id
void Connect_send_RemoteCapability( uint8_t id, uint8_t capabilityIndex, uint8_t state, uint8_t stateType, uint8_t numArgs, uint8_t *args )
{
   // Ignore current id
   if ( id == Connect_id )
      return;

   // Prepare header
   remote_capability_msg.id = id;
   remote_capability_msg.capability_index = capabilityIndex;
   remote_capability_msg.state = state;
   remote_capability_msg.state_type = stateType;
   remote_capability_msg.num_args = numArgs;

   // Send towards slave node
   if ( id > Connect_id )
   {
      // Send message
      upipe_send_variable_msg(&slave_pipe, &remote_capability_msg.header, args, numArgs);
   }

   // Send towards master node
   if ( id < Connect_id || id == 255 )
   {
      // Send message
      upipe_send_variable_msg(&master_pipe, &remote_capability_msg.header, args, numArgs);
   }
}

void Connect_send_Idle( uint8_t num )
{
   upipe_send_idle(&master_pipe, num);
   upipe_send_idle(&slave_pipe, num);
}


// -- Connect receive functions --

// - Cable Check variables -
uint32_t Connect_cableFaultsMaster = 0;
uint32_t Connect_cableFaultsSlave  = 0;
uint32_t Connect_cableChecksMaster = 0;
uint32_t Connect_cableChecksSlave  = 0;
uint8_t  Connect_cableOkMaster = 0;
uint8_t  Connect_cableOkSlave  = 0;

uint8_t Connect_receive_CableCheck( uint8_t byte, uint16_t *pending_bytes, uint8_t uart_num )
{
   // Check if this is the first byte
   if ( *pending_bytes == 0xFFFF )
   {
      *pending_bytes = byte;

      if ( Connect_debug )
      {
         dbug_msg("PENDING SET -> ");
         printHex( byte );
         print(" ");
         printHex( *pending_bytes );
         print( NL );
      }
   }
   // Verify byte
   else
   {
      (*pending_bytes)--;

      // The argument bytes are always 0xD2 (11010010)
      if ( byte != 0xD2 )
      {
         warn_print("Cable Fault!");

         // Check which side of the chain
         if ( uart_num == UART_Slave )
         {
            Connect_cableFaultsSlave++;
            Connect_cableOkSlave = 0;
            print(" Slave ");
         }
         else
         {
            // Lower current requirement during errors
            // USB minimum
            // Only if this is not the master node
            if ( Connect_id != 0 )
            {
               Output_update_external_current( 100 );
            }

            Connect_cableFaultsMaster++;
            Connect_cableOkMaster = 0;
            print(" Master ");
         }
         printHex( byte );
         print( NL );

         // Signal that the command should wait for a SYN again
         return 1;
      }
      else
      {
         // Check which side of the chain
         if ( uart_num == UART_Slave )
         {
            Connect_cableChecksSlave++;
         }
         else
         {
            // If we already have an Id, then set max current again
            if ( Connect_id != 255 && Connect_id != 0 )
            {
               // TODO reset to original negotiated current
               Output_update_external_current( 500 );
            }
            Connect_cableChecksMaster++;
         }
      }
   }

   // If cable check was successful, set cable ok
   if ( *pending_bytes == 0 )
   {
      if ( uart_num == UART_Slave )
      {
         Connect_cableOkSlave = 1;
      }
      else
      {
         Connect_cableOkMaster = 1;
      }
   }

   if ( Connect_debug )
   {
      dbug_msg("CABLECHECK RECEIVE - ");
      printHex( byte );
      print(" ");
      printHex( *pending_bytes );
      print( NL );
   }

   // Check whether the cable check has finished
   return *pending_bytes == 0 ? 1 : 0;
}

uint8_t Connect_receive_IdRequest( uint8_t byte, uint16_t *pending_bytes, uint8_t uart_num )
{
   dbug_print("IdRequest");
   // Check the directionality
   if ( uart_num == UART_Master )
   {
      erro_print("Invalid IdRequest direction...");
   }

   // Check if master, begin IdEnumeration
   if ( Connect_master )
   {
      // The first device is always id 1
      // Id 0 is reserved for the master
      Connect_send_IdEnumeration( 1 );
   }
   // Propagate IdRequest
   else
   {
      Connect_send_IdRequest();
   }

   return 1;
}

uint8_t Connect_receive_IdEnumeration( uint8_t id, uint16_t *pending_bytes, uint8_t uart_num )
{
   dbug_print("IdEnumeration");
   // Check the directionality
   if ( uart_num == UART_Slave )
   {
      erro_print("Invalid IdEnumeration direction...");
   }

   // Set the device id
   Connect_id = id;

   // Send reponse back to master
   Connect_send_IdReport( id );

   // Node now enumerated, set external power to USB Max
   // Only set if this is not the master node
   // TODO Determine power slice for each node as part of protocol
   if ( Connect_id != 0 )
   {
      Output_update_external_current( 500 );
   }

   // Propogate next Id if the connection is ok
   if ( Connect_cableOkSlave )
   {
      Connect_send_IdEnumeration( id + 1 );
   }

   return 1;
}

uint8_t Connect_receive_IdReport( uint8_t id, uint16_t *pending_bytes, uint8_t uart_num )
{
   dbug_print("IdReport");
   // Check the directionality
   if ( uart_num == UART_Master )
   {
      erro_print("Invalid IdRequest direction...");
   }

   // Track Id response if master
   if ( Connect_master )
   {
      info_msg("Id Reported: ");
      printHex( id );
      print( NL );

      // Check if this is the highest ID
      if ( id > Connect_maxId )
         Connect_maxId = id;
      return 1;
   }
   // Propagate id if yet another slave
   else
   {
      Connect_send_IdReport( id );
   }

   return 1;
}

// - Scan Code Variables -
TriggerGuide Connect_receive_ScanCodeBuffer;
uint8_t Connect_receive_ScanCodeBufferPos;
uint8_t Connect_receive_ScanCodeDeviceId;

uint8_t Connect_receive_ScanCode( uint8_t byte, uint16_t *pending_bytes, uint8_t uart_num )
{
   // Check the directionality
   if ( uart_num == UART_Master )
   {
      erro_print("Invalid ScanCode direction...");
   }

   // Master node, trigger scan codes
   if ( Connect_master ) switch ( (*pending_bytes)-- )
   {
   // Byte count always starts at 0xFFFF
   case 0xFFFF: // Device Id
      Connect_receive_ScanCodeDeviceId = byte;
      break;

   case 0xFFFE: // Number of TriggerGuides in bytes (byte * 3)
      *pending_bytes = byte * sizeof( TriggerGuide );
      Connect_receive_ScanCodeBufferPos = 0;
      break;

   default:
      // Set the specific TriggerGuide entry
      ((uint8_t*)&Connect_receive_ScanCodeBuffer)[ Connect_receive_ScanCodeBufferPos++ ] = byte;

      // Reset the BufferPos if higher than sizeof TriggerGuide
      // And send the TriggerGuide to the Macro Module
      if ( Connect_receive_ScanCodeBufferPos >= sizeof( TriggerGuide ) )
      {
         Connect_receive_ScanCodeBufferPos = 0;

         // Adjust ScanCode offset
         if ( Connect_receive_ScanCodeDeviceId > 0 )
         {
            // Check if this node is too large
            if ( Connect_receive_ScanCodeDeviceId >= InterconnectNodeMax )
            {
               warn_msg("Not enough interconnect layout nodes configured: ");
               printHex( Connect_receive_ScanCodeDeviceId );
               print( NL );
               break;
            }

            // This variable is in generatedKeymaps.h
            extern uint8_t InterconnectOffsetList[];
            Connect_receive_ScanCodeBuffer.scanCode = Connect_receive_ScanCodeBuffer.scanCode + InterconnectOffsetList[ Connect_receive_ScanCodeDeviceId - 1 ];
         }

         // ScanCode receive debug
         if ( Connect_debug )
         {
            dbug_msg("");
            printHex( Connect_receive_ScanCodeBuffer.type );
            print(" ");
            printHex( Connect_receive_ScanCodeBuffer.state );
            print(" ");
            printHex( Connect_receive_ScanCodeBuffer.scanCode );
            print( NL );
         }

         // Send ScanCode to macro module
         Macro_pressReleaseAdd( &Connect_receive_ScanCodeBuffer );
      }

      break;
   }
   // Propagate ScanCode packet
   // XXX It would be safer to buffer the scancodes first, before transmitting the packet -Jacob
   //     The current method is the more efficient/aggressive, but could cause issues if there were errors during transmission
   else switch ( (*pending_bytes)-- )
   {
   // Byte count always starts at 0xFFFF
   case 0xFFFF: // Device Id
   {
      Connect_receive_ScanCodeDeviceId = byte;

      // Lock the master Tx buffer
      uart_lockTx( UART_Master );

      // Send header + Id byte
      uint8_t header[] = { 0x16, 0x01, ScanCode, byte };
      Connect_addBytes( header, sizeof( header ), UART_Master );
      break;
   }
   case 0xFFFE: // Number of TriggerGuides in bytes
      *pending_bytes = byte * sizeof( TriggerGuide );
      Connect_receive_ScanCodeBufferPos = 0;

      // Pass through byte
      Connect_addBytes( &byte, 1, UART_Master );
      break;

   default:
      // Pass through byte
      Connect_addBytes( &byte, 1, UART_Master );

      // Unlock Tx Buffer after sending last byte
      if ( *pending_bytes == 0 )
         uart_unlockTx( UART_Master );
      break;
   }

   // Check whether the scan codes have finished sending
   return *pending_bytes == 0 ? 1 : 0;
}

uint8_t Connect_receive_Animation( uint8_t byte, uint16_t *pending_bytes, uint8_t uart_num )
{
   dbug_print("Animation");
   return 1;
}

// - Remote Capability Variables -
#define Connect_receive_RemoteCapabilityMaxArgs 25 // XXX Calculate the max using kll
RemoteCapabilityCommand Connect_receive_RemoteCapabilityBuffer;
uint8_t Connect_receive_RemoteCapabilityArgs[Connect_receive_RemoteCapabilityMaxArgs];

uint8_t Connect_receive_RemoteCapability( uint8_t byte, uint16_t *pending_bytes, uint8_t uart_num )
{
   // Check which byte in the packet we are at
   switch ( (*pending_bytes)-- )
   {
   // Byte count always starts at 0xFFFF
   case 0xFFFF: // Device Id
      Connect_receive_RemoteCapabilityBuffer.id = byte;
      break;

   case 0xFFFE: // Capability Index
      Connect_receive_RemoteCapabilityBuffer.capabilityIndex = byte;
      break;

   case 0xFFFD: // State
      Connect_receive_RemoteCapabilityBuffer.state = byte;
      break;

   case 0xFFFC: // StateType
      Connect_receive_RemoteCapabilityBuffer.stateType = byte;
      break;

   case 0xFFFB: // Number of args
      Connect_receive_RemoteCapabilityBuffer.numArgs = byte;
      *pending_bytes = byte;
      break;

   default:     // Args (# defined by previous byte)
      Connect_receive_RemoteCapabilityArgs[
         Connect_receive_RemoteCapabilityBuffer.numArgs - *pending_bytes + 1
      ] = byte;

      // If entire packet has been fully received
      if ( *pending_bytes == 0 )
      {
         // Determine if this is the node to run the capability on
         // Conditions: Matches or broadcast (0xFF)
         if ( Connect_receive_RemoteCapabilityBuffer.id == 0xFF
            || Connect_receive_RemoteCapabilityBuffer.id == Connect_id )
         {
            extern const Capability CapabilitiesList[]; // See generatedKeymap.h
            void (*capability)(uint8_t, uint8_t, uint8_t*) = (void(*)(uint8_t, uint8_t, uint8_t*))(
               CapabilitiesList[ Connect_receive_RemoteCapabilityBuffer.capabilityIndex ].func
            );
            capability(
               Connect_receive_RemoteCapabilityBuffer.state,
               Connect_receive_RemoteCapabilityBuffer.stateType,
               &Connect_receive_RemoteCapabilityArgs[2]
            );
         }

         // If this is not the correct node, keep sending it in the same direction (doesn't matter if more nodes exist)
         // or if this is a broadcast
         if ( Connect_receive_RemoteCapabilityBuffer.id == 0xFF
            || Connect_receive_RemoteCapabilityBuffer.id != Connect_id )
         {
            // Prepare outgoing packet
            Connect_receive_RemoteCapabilityBuffer.command = RemoteCapability;

            // Send to the other UART (not the one receiving the packet from
            uint8_t uart_direction = uart_num == UART_Master ? UART_Slave : UART_Master;

            // Lock Tx UART
            switch ( uart_direction )
            {
            case UART_Master: uart_lockTx( UART_Master ); break;
            case UART_Slave:  uart_lockTx( UART_Slave );  break;
            }

            // Send header
            uint8_t header[] = { 0x16, 0x01 };
            Connect_addBytes( header, sizeof( header ), uart_direction );

            // Send Remote Capability and arguments
            Connect_addBytes( (uint8_t*)&Connect_receive_RemoteCapabilityBuffer, sizeof( RemoteCapabilityCommand ), uart_direction );
            Connect_addBytes( Connect_receive_RemoteCapabilityArgs, Connect_receive_RemoteCapabilityBuffer.numArgs, uart_direction );

            // Unlock Tx UART
            switch ( uart_direction )
            {
            case UART_Master: uart_unlockTx( UART_Master ); break;
            case UART_Slave:  uart_unlockTx( UART_Slave );  break;
            }
         }
      }
      break;
   }

   // Check whether the scan codes have finished sending
   return *pending_bytes == 0 ? 1 : 0;
}


// Baud Rate
// NOTE: If finer baud adjustment is needed see UARTx_C4 -> BRFA in the datasheet
uint16_t Connect_baud = UARTConnectBaud_define; // Max setting of 8191
uint16_t Connect_baudFine = UARTConnectBaudFine_define;

// Connect receive function lookup
void *Connect_receiveFunctions[] = {
   Connect_receive_CableCheck,
   Connect_receive_IdRequest,
   Connect_receive_IdEnumeration,
   Connect_receive_IdReport,
   Connect_receive_ScanCode,
   Connect_receive_Animation,
   Connect_receive_RemoteCapability,
};



// ----- Functions -----

// Resets the state of the UART buffers and state variables
void Connect_reset()
{
   upipe_reset(&slave_pipe, UART_Buffer_Size);
   upipe_reset(&master_pipe, UART_Buffer_Size);

   // Reset Rx
   memset( (void*)uart_rx_status, 0, sizeof( UARTStatusRx ) * UART_Num_Interfaces );

   // Reset Tx
   memset( (void*)uart_tx_status, 0, sizeof( UARTStatusTx ) * UART_Num_Interfaces );

   // Set Rx/Tx buffers as ready
   for ( uint8_t inter = 0; inter < UART_Num_Interfaces; inter++ )
   {
      uart_tx_status[ inter ].status = UARTStatus_Ready;
      uart_rx_buf[ inter ]->last_read = UART_Buffer_Size;
   }
}

void Connect_set_master( uint8_t master )
{
   // Check if master
   Connect_master = master;
   if ( Connect_master )
      Connect_id = 0; // 0x00 is always the master Id
   else
      Connect_id = 0xFF;
}

// Setup connection to other side
// - Only supports a single slave and master
// - If USB has been initiallized at this point, this side is the master
// - If both sides assert master, flash error leds
void Connect_setup( uint8_t master )
{
   // Indication that UARTs are not ready
   uarts_configured = 0;

   // Register Connect CLI dictionary
   CLI_registerDictionary( uartConnectCLIDict, uartConnectCLIDictName );

   // Check if master
   Connect_set_master(master);

   error_code_t err;
   err = upipe_init(&slave_pipe, UART_Slave, Connect_baud, UART_Buffer_Size);
   if (err != SUCCESS)
   {
      erro_print("Failed to setup slave uart...");
      printHex32(err);
      return;
   }

   err = upipe_init(&master_pipe, UART_Master, Connect_baud, UART_Buffer_Size);
   if (err != SUCCESS)
   {
      erro_print("Failed to setup slave uart...");
      printHex32(err);
      return;
   }

   slave_handle = slave_pipe.uart_handle;
   master_handle = master_pipe.uart_handle;
   uart_rx_buf[UART_Slave] = &slave_pipe.rx_buf;
   uart_rx_buf[UART_Master] = &master_pipe.rx_buf;

   // UARTs are now ready to go
   uarts_configured = 1;

   // Reset the state of the UART variables
   Connect_reset();
}


#define DMA_BUF_POS( x, pos ) \
   case x: \
      pos = DMA_TCD##x##_CITER_ELINKNO; \
      break
void Connect_rx_process( uint8_t uartNum )
{
   // Determine current position to read until
   uint16_t bufpos = 0;
   switch ( uartNum )
   {
   DMA_BUF_POS( 0, bufpos );
   DMA_BUF_POS( 1, bufpos );
   }

   // Process each of the new bytes
   // Even if we receive more bytes during processing, wait until the next check so we don't starve other tasks
   while ( bufpos != uart_rx_buf[ uartNum ]->last_read )
   {
      // If the last_read byte is at the buffer edge, roll back to beginning
      if ( uart_rx_buf[ uartNum ]->last_read == 0 )
      {
         uart_rx_buf[ uartNum ]->last_read = UART_Buffer_Size;

         // Check to see if we're at the boundary
         if ( bufpos == UART_Buffer_Size )
            break;
      }

      // Read the byte out of Rx DMA buffer
      uint8_t byte = uart_rx_buf[ uartNum ]->buffer[ UART_Buffer_Size - uart_rx_buf[ uartNum ]->last_read-- ];

      if ( Connect_debug )
      {
         printHex( byte );
         print(" ");
      }

      // Process UART byte
      switch ( uart_rx_status[ uartNum ].status )
      {
      // Every packet must start with a SYN / 0x16
      case UARTStatus_Wait:
         if ( Connect_debug )
         {
            print(" Wait ");
         }
         uart_rx_status[ uartNum ].status = byte == 0x16 ? UARTStatus_SYN : UARTStatus_Wait;
         break;

      // After a SYN, there must be a SOH / 0x01
      case UARTStatus_SYN:
         if ( Connect_debug )
         {
            print(" SYN ");
         }
         uart_rx_status[ uartNum ].status = byte == 0x01 ? UARTStatus_SOH : UARTStatus_Wait;
         break;

      case UARTStatus_SOH:
         if ( Connect_debug )
         {
            print(" Data ");
         }
         uart_rx_status[ uartNum ].status = byte > 0x00 ? UARTStatus_Data : UARTStatus_Wait;
         break;

      // After a SOH the packet structure may diverge a bit
      // This is the packet type field (refer to the Command enum)
      // For very small packets (e.g. IdRequest) this is all that's required to take action
      case UARTStatus_Data:
      {
         if ( Connect_debug )
         {
            print(" SOH ");
         }

         // Check if this is actually a reserved CMD 0x16 (Error condition)
         if ( byte == Command_SYN )
         {
            uart_rx_status[ uartNum ].status = UARTStatus_SYN;
            break;
         }

         // Otherwise process the command
         if ( byte < Command_TOP )
         {
            uart_rx_status[ uartNum ].status = UARTStatus_Command;
            uart_rx_status[ uartNum ].command = byte;
            uart_rx_status[ uartNum ].bytes_waiting = 0xFFFF;
         }
         // Invalid packet type, ignore
         else
         {
            uart_rx_status[ uartNum ].status = UARTStatus_Wait;
         }

         // Check if this is a very short packet
         switch ( uart_rx_status[ uartNum ].command )
         {
         case IdRequest:
            Connect_receive_IdRequest( 0, (uint16_t*)&uart_rx_status[ uartNum ].bytes_waiting, uartNum );
            uart_rx_status[ uartNum ].status = UARTStatus_Wait;
            break;

         default:
            if ( Connect_debug )
            {
               print(" ### ");
               printHex( uart_rx_status[ uartNum ].command );
            }
            break;
         }
         break;
      }

      // After the packet type has been deciphered do Command specific processing
      // Until the Command has received all the bytes it requires the UART buffer stays in this state
      case UARTStatus_Command:
      {
         if ( Connect_debug )
         {
            print(" CMD ");
         }
         /* Call specific UARTConnect command receive function */
         uint8_t (*rcvFunc)(uint8_t, uint16_t(*), uint8_t) = (uint8_t(*)(uint8_t, uint16_t(*), uint8_t))(Connect_receiveFunctions[ uart_rx_status[ uartNum ].command ]);
         if ( rcvFunc( byte, (uint16_t*)&uart_rx_status[ uartNum ].bytes_waiting, uartNum ) )
            uart_rx_status[ uartNum ].status = UARTStatus_Wait;
         break;
      }

      // Unknown status, should never get here
      default:
         erro_msg("Invalid UARTStatus...");
         uart_rx_status[ uartNum ].status = UARTStatus_Wait;
         continue;
      }

      if ( Connect_debug )
      {
         print( NL );
      }
   }
}


// Scan for updates in the master/slave
// - Interrupts will deal with most input functions
// - Used to send queries
// - SyncEvent is sent immediately once the current command is sent
// - SyncEvent is also blocking until sent
void Connect_scan()
{
   // Check if initially configured as a slave and usb comes up
   // Then reconfigure as a master
   if ( !Connect_master && Output_Available && !Connect_override )
   {
      Connect_set_master( Output_Available );
      Connect_reset();
   }

   // Limit how often we do cable checks
   //uint32_t time_compare = 0x007; // Used for debugging cables -HaaTa
   uint32_t time_compare = 0x7FF; // Must be all 1's, 0x3FF is valid, 0x4FF is not
   uint32_t current_time = systick_millis_count;
   if ( Connect_lastCheck != current_time
      && ( current_time & time_compare ) == time_compare
   )
   {
      // Make sure we don't double check if the clock speed is too high
      Connect_lastCheck = current_time;

      // Send a cable check command of 2 bytes
      Connect_send_CableCheck( UARTConnectCableCheckLength_define );

      // If this is a slave, and we don't have an id yeth
      // Don't bother sending if there are cable issues
      if ( !Connect_master && Connect_id == 0xFF && Connect_cableOkMaster )
      {
         Connect_send_IdRequest();
      }
   }

   // Only process commands if uarts have been configured
   if ( uarts_configured )
   {
      // Check if Tx Buffers are empty and the Tx Ring buffers have data to send
      // This happens if there was previously nothing to send
      //uart_fillTxFifo(UART_Slave, slave_handle);
      //uart_fillTxFifo(UART_Master, master_handle);
      upipe_process(&slave_pipe);
      upipe_process(&master_pipe);

      // Process Rx Buffers
      Connect_rx_process( 0 );
      Connect_rx_process( 1 );
   }
}


// Called by parent Scan module whenever the available current changes
void Connect_currentChange( unsigned int current )
{
   // TODO - Any potential power saving here?
}



// ----- CLI Command Functions -----

void cliFunc_connectCmd( char* args )
{
   // Parse number from argument
   //  NOTE: Only first argument is used
   char* arg1Ptr;
   char* arg2Ptr;
   CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

   print( NL );

   switch ( numToInt( &arg1Ptr[0] ) )
   {
   case CableCheck:
      Connect_send_CableCheck( UARTConnectCableCheckLength_define );
      break;

   case IdRequest:
      Connect_send_IdRequest();
      break;

   case IdEnumeration:
      Connect_send_IdEnumeration( 5 );
      break;

   case IdReport:
      Connect_send_IdReport( 8 );
      break;

   case ScanCode:
   {
      TriggerGuide scanCodes[] = { { 0x00, 0x01, 0x05 }, { 0x00, 0x03, 0x16 } };
      Connect_send_ScanCode( 10, scanCodes, 2 );
      break;
   }
   case Animation:
      break;

   case RemoteCapability:
      // TODO
      break;

   case RemoteOutput:
      // TODO
      break;

   case RemoteInput:
      // TODO
      break;

   default:
      break;
   }
}

void cliFunc_connectDbg( char* args )
{
   print( NL );
   info_msg("Connect Debug Mode Toggle");
   Connect_debug = !Connect_debug;
}

void cliFunc_connectIdl( char* args )
{
   // Parse number from argument
   //  NOTE: Only first argument is used
   char* arg1Ptr;
   char* arg2Ptr;
   CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

   print( NL );
   info_msg("Sending Sync Idles...");

   uint8_t count = numToInt( &arg1Ptr[0] );
   // Default to 2 idles
   if ( count == 0 )
      count = 2;

   Connect_send_Idle( count );
}

void cliFunc_connectLst( char* args )
{
   const char *Command_strs[] = {
      "CableCheck",
      "IdRequest",
      "IdEnumeration",
      "IdReport",
      "ScanCode",
      "Animation",
      "RemoteCapability",
      "RemoteOutput",
      "RemoteInput",
   };

   print( NL );
   info_msg("List of UARTConnect commands");
   for ( uint8_t cmd = 0; cmd < Command_TOP; cmd++ )
   {
      print( NL );
      printInt8( cmd );
      print(" - ");
      dPrint( (char*)Command_strs[ cmd ] );
   }
}

void cliFunc_connectMst( char* args )
{
   // Parse number from argument
   //  NOTE: Only first argument is used
   char* arg1Ptr;
   char* arg2Ptr;
   CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

   print( NL );

   // Set override
   Connect_override = 1;

   switch ( arg1Ptr[0] )
   {
   // Disable override
   case 'd':
   case 'D':
      Connect_override = 0;
   case 's':
   case 'S':
      info_msg("Setting device as slave.");
      Connect_master = 0;
      Connect_id = 0xFF;
      break;

   case 'm':
   case 'M':
   default:
      info_msg("Setting device as master.");
      Connect_master = 1;
      Connect_id = 0;
      break;
   }
}

void cliFunc_connectRst( char* args )
{
   print( NL );
   info_msg("Resetting UARTConnect state...");
   Connect_reset();

   // Reset node id
   Connect_id = 0xFF;
}

void cliFunc_connectSts( char* args )
{
   print( NL );
   info_msg("UARTConnect Status");
   print( NL "Device Type:\t" );
   print( Connect_master ? "Master" : "Slave" );
   print( NL "Device Id:\t" );
   printHex( Connect_id );
   print( NL "Max Id:\t" );
   printHex( Connect_maxId );
   print( NL "Master <=" NL "\tStatus:\t");
   printHex( Connect_cableOkMaster );
   print( NL "\tFaults:\t");
   printHex32( Connect_cableFaultsMaster );
   print("/");
   printHex32( Connect_cableChecksMaster );
   print( NL "\tRx:\t");
   printHex( uart_rx_status[UART_Master].status );
   print( NL "\tTx:\t");
   printHex( uart_tx_status[UART_Master].status );
   print( NL "Slave <=" NL "\tStatus:\t");
   printHex( Connect_cableOkSlave );
   print( NL "\tFaults:\t");
   printHex32( Connect_cableFaultsSlave );
   print("/");
   printHex32( Connect_cableChecksSlave );
   print( NL "\tRx:\t");
   printHex( uart_rx_status[UART_Slave].status );
   print( NL "\tTx:\t");
   printHex( uart_tx_status[UART_Slave].status );
}

