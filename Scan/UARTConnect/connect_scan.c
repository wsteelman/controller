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
#include <connect_scan.h>
#include <interconnect.h>

// ----- Function Declarations -----

// CLI Functions
void cliFunc_connectCmd ( char *args );
void cliFunc_connectDbg ( char *args );
void cliFunc_connectLst ( char *args );

// ----- Messages -----

scan_code_msg_t         scan_code_msg           = { {CmdCommand_SYN, 0x01, sizeof(scan_code_msg_t), CmdScanCode}, 0xFF, 0};
animation_msg_t         animation_msg           = { {CmdCommand_SYN, 0x01, sizeof(animation_msg_t), CmdAnimation}, 0xFF, 0};
remote_capability_msg_t remote_capability_msg   = { {CmdCommand_SYN, 0x01, sizeof(remote_capability_msg_t), CmdRemoteCapability}, 0xFF, 0, 0, 0, 0};
enable_master_msg_t     enable_master_msg       = { {CmdCommand_SYN, 0x01, sizeof(enable_master_msg_t), CmdEnableMaster}, 0xFF, usb_protocol};

// ----- Variables -----

// Connect Module command dictionary
CLIDict_Entry( connectCmd,  "Sends a command via UART Connect, first arg is which uart, next arg is the command, rest are the arguments." );
CLIDict_Entry( connectDbg,  "Toggle UARTConnect debug mode." );
CLIDict_Entry( connectLst,  "Lists available UARTConnect commands and index id" );
CLIDict_Def( uartConnectCLIDict, "UARTConnect Module Commands" ) = {
   CLIDict_Item( connectCmd ),
   CLIDict_Item( connectDbg ),
   CLIDict_Item( connectLst ),
   { 0, 0, 0 } // Null entry for dictionary end
};


// -- Connect Device Id Variables --
uint8_t Connect_id = 255; // Invalid, unset
uint8_t Connect_master = 0;
uint8_t Connect_master_override = 0;

// -- Control Variables --
uint8_t Connect_debug = 0;      // Set 1 for debug
uint8_t Connect_override = 0;   // Prevents master from automatically being set

// -- Connect send functions --

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
   Iconnect_send_variable_msg(Iconnect_up, &scan_code_msg.header, (uint8_t*)scanCodeStateList, code_bytes);
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
   Iconnect_send_variable_msg(Iconnect_down, &animation_msg.header, paramList, numParams);
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
      Iconnect_send_variable_msg(Iconnect_down, &remote_capability_msg.header, args, numArgs);
   }

   // Send towards master node
   if ( id < Connect_id || id == 255 )
   {
      // Send message
      Iconnect_send_variable_msg(Iconnect_up, &remote_capability_msg.header, args, numArgs);
   }
}

void Connect_send_EnableMaster ( uint8_t id, protocol_t p )
{
   enable_master_msg.id = id;
   enable_master_msg.output_protocol = p;
   Iconnect_send_msg(Iconnect_down, &enable_master_msg.header);
}

// -- Connect receive functions --


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
      Iconnect_send_msg(Iconnect_up, hdr);

   return SUCCESS;
}

uint8_t Connect_receive_Animation( const msg_header *hdr )
{
   dbug_print("Animation");
   return SUCCESS;
}

uint8_t Connect_receive_RemoteCapability( const msg_header * hdr, Iconnect_direction opposite_pipe)
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
      Iconnect_send_msg(opposite_pipe, hdr);
   }

   return SUCCESS;
}

uint8_t Connect_receive_slave_RemoteCapability( const msg_header * hdr )
{
   return Connect_receive_RemoteCapability(hdr, Iconnect_up);
}
uint8_t Connect_receive_master_RemoteCapability( const msg_header * hdr )
{
   return Connect_receive_RemoteCapability(hdr, Iconnect_down);
}

uint8_t Connect_receive_master_EnableMaster( const msg_header *hdr )
{
   dbug_print("EnableMaster");

   error_code_t err = SUCCESS;
   const enable_master_msg_t *msg = (const enable_master_msg_t*)hdr;
   // if message is for this node, enable master capability
   if (msg->id == Connect_id)
      Connect_master_override = 1;
   // else forward to next node in the chain
   else
      err = Iconnect_send_msg(Iconnect_down, hdr);

   return err;
}

uint8_t Connect_receive_slave_EnableMaster( const msg_header *hdr )
{
   dbug_print("EnableMaster");

   // forward ack upstream
   //return Iconnect_send_msg(Iconnect_up, hdr);
   return SUCCESS;
}

// ----- Functions -----

// Setup connection to other side
// - Only supports a single slave and master
// - If USB has been initiallized at this point, this side is the master
// - If both sides assert master, flash error leds
void Connect_setup( uint8_t master )
{
   // Register Connect CLI dictionary
   CLI_registerDictionary( uartConnectCLIDict, uartConnectCLIDictName );

   // register slave pipe callbacks
   Iconnect_register_callback(Iconnect_down, CmdScanCode,           &Connect_receive_slave_ScanCode);
   Iconnect_register_callback(Iconnect_down, CmdAnimation,          &Connect_receive_Animation);
   Iconnect_register_callback(Iconnect_down, CmdRemoteCapability,   &Connect_receive_slave_RemoteCapability);
   Iconnect_register_callback(Iconnect_down, CmdEnableMaster,       &Connect_receive_slave_EnableMaster);

   // register master pipe callbacks
   Iconnect_register_callback(Iconnect_up, CmdScanCode,          &Connect_receive_slave_ScanCode);
   Iconnect_register_callback(Iconnect_up, CmdAnimation,         &Connect_receive_Animation);
   Iconnect_register_callback(Iconnect_up, CmdRemoteCapability,  &Connect_receive_master_RemoteCapability);
   Iconnect_register_callback(Iconnect_up, CmdEnableMaster,      &Connect_receive_master_EnableMaster);
}

// Scan for updates in the master/slave
// - Interrupts will deal with most input functions
// - Used to send queries
// - SyncEvent is sent immediately once the current command is sent
// - SyncEvent is also blocking until sent
void Connect_scan()
{
   Connect_master = (connect_node.master || Connect_master_override) && Output_available();
   Connect_id = connect_node.id;
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
   case CmdScanCode:
   {
      TriggerGuide scanCodes[] = { { 0x00, 0x01, 0x05 }, { 0x00, 0x03, 0x16 } };
      Connect_send_ScanCode( 10, scanCodes, 2 );
      break;
   }
   case CmdAnimation:
      break;

   case CmdRemoteCapability:
      // TODO
      break;

   case CmdRemoteOutput:
      // TODO
      break;

   case CmdRemoteInput:
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

void cliFunc_connectLst( char* args )
{
   // TODO: need to fix command indicies
   const char *Command_strs[] = {
      "ScanCode",
      "Animation",
      "RemoteCapability",
      "RemoteOutput",
      "RemoteInput",
   };

   print( NL );
   info_msg("List of UARTConnect commands");
   for ( uint8_t cmd = 0; cmd < 5; cmd++ )
   {
      print( NL );
      printInt8( cmd );
      print(" - ");
      dPrint( (char*)Command_strs[ cmd ] );
   }
}

