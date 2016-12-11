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

typedef enum protocol
{
   default_protocol,
   usb_protocol,
   uart_protocol,
} protocol;
// ----- Messages -----

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

typedef struct scan_code_msg_t
{
   msg_header     header;
   uint8_t        id;
   uint8_t        count;
   TriggerGuide   codes[];
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

typedef struct enable_master_msg_t
{
   msg_header     header;
   uint8_t        id;
   protocol       output_protocol;
} enable_master_msg_t;

cable_check_msg_t       cable_check_msg         = { {CmdCommand_SYN, 0x01, sizeof(cable_check_msg_t), CmdCableCheck}, UARTConnectCableCheckLength_define, {0xD2}};
id_request_msg_t        id_request_msg          = { {CmdCommand_SYN, 0x01, sizeof(id_request_msg_t), CmdIdRequest}, 0x00 };
id_enum_msg_t           id_enum_msg             = { {CmdCommand_SYN, 0x01, sizeof(id_enum_msg_t), CmdIdEnumeration}, 0xFF};
id_report_msg_t         id_report_msg           = { {CmdCommand_SYN, 0x01, sizeof(id_report_msg_t), CmdIdReport}, 0xFF};
scan_code_msg_t         scan_code_msg           = { {CmdCommand_SYN, 0x01, sizeof(scan_code_msg_t), CmdScanCode}, 0xFF, 0};
animation_msg_t         animation_msg           = { {CmdCommand_SYN, 0x01, sizeof(animation_msg_t), CmdAnimation}, 0xFF, 0};
remote_capability_msg_t remote_capability_msg   = { {CmdCommand_SYN, 0x01, sizeof(remote_capability_msg_t), CmdRemoteCapability}, 0xFF, 0, 0, 0, 0};
enable_master_msg_t     enable_master_msg       = { {CmdCommand_SYN, 0x01, sizeof(enable_master_msg_t), CmdEnableMaster}, 0xFF, default_protocol};

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

// -- UART Variables --
uart_message_pipe_t slave_pipe;
uart_message_pipe_t master_pipe;

void Connect_set_master( uint8_t master, uint8_t id )
{
   // Check if master
   Connect_master = master;
   Connect_id = id; // 0x00 is always the master Id
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

void Connect_send_EnableMaster ( uint8_t id, protocol p )
{
   enable_master_msg.id = id;
   enable_master_msg.output_protocol = p;
   upipe_send_msg(&slave_pipe, &enable_master_msg.header);
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

uint8_t Connect_receive_slave_CableCheck( const msg_header* hdr )
{
   const cable_check_msg_t *msg = (cable_check_msg_t*)hdr;
   for (uint8_t i = 0; i < msg->len; ++i)
   {
      if (msg->pattern[i] != 0xD2)
      {
         warn_print("Cable Fault!");

         Connect_cableFaultsSlave++;
         Connect_cableOkSlave = 0;
         print(" Slave ");
         print( NL );

         return SUCCESS;
      }
      else
         Connect_cableChecksSlave++;
   }

   Connect_cableOkSlave = 1;

   if ( Connect_debug )
   {
      dbug_msg("CABLECHECK RECEIVE");
   }
   return SUCCESS;

}

uint8_t Connect_receive_master_CableCheck( const msg_header* hdr )
{
   const cable_check_msg_t *msg = (cable_check_msg_t*)hdr;
   for (uint8_t i = 0; i < msg->len; ++i)
   {
      if (msg->pattern[i] != 0xD2)
      {
         warn_print("Cable Fault!");

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
         print( NL );

         return SUCCESS;
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

   Connect_cableOkMaster = 1;

   if ( Connect_debug )
   {
      dbug_msg("CABLECHECK RECEIVE");
   }
   return SUCCESS;
}

uint8_t Connect_receive_slave_IdRequest( const msg_header *hdr )
{
   dbug_print("IdRequest");

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
   return SUCCESS;
}


uint8_t Connect_receive_master_IdRequest( const msg_header *hdr )
{
   dbug_print("IdRequest");
   erro_print("Invalid IdRequest direction...");
   return SUCCESS;
}

uint8_t Connect_receive_slave_IdEnumeration( const msg_header *hdr )
{
   dbug_print("IdEnumeration");
   erro_print("Invalid IdEnumeration direction...");
   return SUCCESS;
}

uint8_t Connect_receive_master_IdEnumeration( const msg_header *hdr )
{
   dbug_print("IdEnumeration");

   const id_enum_msg_t *msg = (const id_enum_msg_t*)hdr;

   // Set the device id
   Connect_id = msg->id;

   // Send reponse back to master
   Connect_send_IdReport( msg->id );

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
      Connect_send_IdEnumeration( msg->id + 1 );
   }

   return SUCCESS;
}

uint8_t Connect_receive_IdReport( const msg_header *hdr )
{
   dbug_print("IdReport");

   const id_report_msg_t *msg = (const id_report_msg_t*)hdr;

   // Track Id response if master
   if ( Connect_master )
   {
      info_msg("Id Reported: ");
      printHex( msg->id );
      print( NL );

      // Check if this is the highest ID
      if ( msg->id > Connect_maxId )
         Connect_maxId = msg->id;
      return SUCCESS;
   }
   // Propagate id if yet another slave
   else
   {
      Connect_send_IdReport( msg->id );
   }

   return SUCCESS;
}

uint8_t Connect_receive_master_ScanCode( const msg_header *hdr )
{
   erro_print("Invalid ScanCode direction...");
   return SUCCESS;
}

uint8_t Connect_receive_slave_ScanCode( const msg_header *hdr )
{
   const scan_code_msg_t *msg = (const scan_code_msg_t *)hdr;

   // Master node, trigger scan codes
   if ( Connect_master )
   {
      for (uint8_t i = 0; i < msg->count; ++i)
      {
         TriggerGuide guide = msg->codes[i];
         // Adjust ScanCode offset
         if ( msg->id > 0 )
         {
            // Check if this node is too large
            if ( msg->id >= InterconnectNodeMax )
            {
               warn_msg("Not enough interconnect layout nodes configured: ");
               printHex( msg->id );
               print( NL );
               break;
            }

            // This variable is in generatedKeymaps.h
            extern uint8_t InterconnectOffsetList[];
            guide.scanCode += InterconnectOffsetList[ msg->id - 1 ];
         }

         // ScanCode receive debug
         if ( Connect_debug )
         {
            dbug_msg("");
            printHex( guide.type );
            print(" ");
            printHex( guide.state );
            print(" ");
            printHex( guide.scanCode );
            print( NL );
         }

         // Send ScanCode to macro module
         Macro_pressReleaseAdd( &guide );
      }
   }
   // Propagate ScanCode packet
   else
      upipe_send_msg(&master_pipe, hdr);

   return SUCCESS;
}

uint8_t Connect_receive_Animation( const msg_header *hdr )
{
   dbug_print("Animation");
   return SUCCESS;
}

uint8_t Connect_receive_RemoteCapability( const msg_header * hdr, uart_message_pipe_t *opposite_pipe)
{
   const remote_capability_msg_t *msg = (const remote_capability_msg_t *) hdr;
   // Determine if this is the node to run the capability on
   // Conditions: Matches or broadcast (0xFF)
   if ( msg->id == 0xFF || msg->id == Connect_id )
   {
      extern const Capability CapabilitiesList[]; // See generatedKeymap.h
      void (*capability)(uint8_t, uint8_t, const uint8_t*) = (void(*)(uint8_t, uint8_t, const uint8_t*))(
         CapabilitiesList[ msg->capability_index ].func
      );
      capability(msg->state, msg->state_type, &msg->args[0]);
   }

   // If this is not the correct node, keep sending it in the same direction (doesn't matter if more nodes exist)
   // or if this is a broadcast
   if ( msg->id == 0xFF || msg->id != Connect_id )
   {
      upipe_send_msg(opposite_pipe, hdr);
   }

   return SUCCESS;
}

uint8_t Connect_receive_slave_RemoteCapability( const msg_header * hdr )
{
   return Connect_receive_RemoteCapability(hdr, &master_pipe);
}
uint8_t Connect_receive_master_RemoteCapability( const msg_header * hdr )
{
   return Connect_receive_RemoteCapability(hdr, &slave_pipe);
}

uint8_t Connect_receive_master_EnableMaster( const msg_header *hdr )
{
   dbug_print("EnableMaster");

   error_code_t err = SUCCESS;
   const enable_master_msg_t *msg = (const enable_master_msg_t*)hdr;
   // if message is for this node, enable master capability
   if (msg->id == Connect_id)
   {
      Connect_set_master( 1, msg->id );
      // notify output module of any protocol change
      
      // send ack upstream
   }
   // else forward to next node in the chain
   else
      err = upipe_send_msg(&slave_pipe, hdr);

   return err;
}

uint8_t Connect_receive_slave_EnableMaster( const msg_header *hdr )
{
   dbug_print("EnableMaster");

   // forward ack upstream
   return upipe_send_msg(&master_pipe, hdr);
}


// Baud Rate
// NOTE: If finer baud adjustment is needed see UARTx_C4 -> BRFA in the datasheet
uint16_t Connect_baud = UARTConnectBaud_define; // Max setting of 8191
uint16_t Connect_baudFine = UARTConnectBaudFine_define;

// ----- Functions -----

// Resets the state of the UART buffers and state variables
void Connect_reset()
{
   upipe_reset(&slave_pipe, UART_Buffer_Size);
   upipe_reset(&master_pipe, UART_Buffer_Size);
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
   Connect_set_master(master, master ? 0x00 : 0xFF);

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

   // register slave pipe callbacks
   upipe_register_callback(&slave_pipe, CmdCableCheck,         &Connect_receive_slave_CableCheck);
   upipe_register_callback(&slave_pipe, CmdIdRequest,          &Connect_receive_slave_IdRequest);
   upipe_register_callback(&slave_pipe, CmdIdEnumeration,      &Connect_receive_slave_IdEnumeration);
   upipe_register_callback(&slave_pipe, CmdIdReport,           &Connect_receive_IdReport);
   upipe_register_callback(&slave_pipe, CmdScanCode,           &Connect_receive_slave_ScanCode);
   upipe_register_callback(&slave_pipe, CmdAnimation,          &Connect_receive_Animation);
   upipe_register_callback(&slave_pipe, CmdRemoteCapability,   &Connect_receive_slave_RemoteCapability);
   upipe_register_callback(&slave_pipe, CmdEnableMaster,       &Connect_receive_slave_EnableMaster);

   // register master pipe callbacks
   upipe_register_callback(&master_pipe, CmdCableCheck,        &Connect_receive_master_CableCheck);
   upipe_register_callback(&master_pipe, CmdIdRequest,         &Connect_receive_master_IdRequest);
   upipe_register_callback(&master_pipe, CmdIdEnumeration,     &Connect_receive_master_IdEnumeration);
   upipe_register_callback(&master_pipe, CmdIdReport,          &Connect_receive_IdReport);
   upipe_register_callback(&master_pipe, CmdScanCode,          &Connect_receive_slave_ScanCode);
   upipe_register_callback(&master_pipe, CmdAnimation,         &Connect_receive_Animation);
   upipe_register_callback(&master_pipe, CmdRemoteCapability,  &Connect_receive_master_RemoteCapability);
   upipe_register_callback(&master_pipe, CmdEnableMaster,      &Connect_receive_master_EnableMaster);

   // UARTs are now ready to go
   uarts_configured = 1;

   // Reset the state of the UART variables
   Connect_reset();
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
      Connect_set_master( Output_Available, 0x00 );
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
      Connect_send_CableCheck();

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
      upipe_process(&slave_pipe);
      upipe_process(&master_pipe);
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
      Connect_send_CableCheck();
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
   print( NL "Slave <=" NL "\tStatus:\t");
   printHex( Connect_cableOkSlave );
   print( NL "\tFaults:\t");
   printHex32( Connect_cableFaultsSlave );
   print("/");
   printHex32( Connect_cableChecksSlave );
}

