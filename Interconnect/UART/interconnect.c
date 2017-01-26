// Compiler Includes

// Project Includes
#include <cli.h>
#include <kll_defs.h>
#include <print.h>
#include <Lib/delay.h>

#include "interconnect.h"

#define UART_Buffer_Size UARTConnectBufSize_define

Iconnect_node_t connect_node = {0};

uint16_t Iconnect_baud = UARTConnectBaud_define; // Max setting of 8191
uint16_t Iconnect_baudFine = UARTConnectBaudFine_define;
uint32_t Iconnect_last_time = 0; // Cable Check scheduler
uint8_t Iconnect_debug = 0;      // Set 1 for debug


// CLI Functions
void cliFunc_iconnectCmd ( char *args );
void cliFunc_iconnectDbg ( char *args );
void cliFunc_iconnectIdl ( char *args );
void cliFunc_iconnectLst ( char *args );
void cliFunc_iconnectMst ( char *args );
void cliFunc_iconnectRst ( char *args );
void cliFunc_iconnectSts ( char *args );


// Connect Module command dictionary
CLIDict_Entry( iconnectCmd,  "Sends a command via Iconnect, first arg is which uart, next arg is the command, rest are the arguments." );
CLIDict_Entry( iconnectDbg,  "Toggle Iconnect debug mode." );
CLIDict_Entry( iconnectIdl,  "Sends N number of Idle commands, 2 is the default value, and should be sufficient in most cases." );
CLIDict_Entry( iconnectLst,  "Lists available Iconnect commands and index id" );
CLIDict_Entry( iconnectMst,  "Sets the device as master. Use argument of s to set as slave." );
CLIDict_Entry( iconnectRst,  "Resets both Rx and Tx connect buffers and state variables." );
CLIDict_Entry( iconnectSts,  "Iconnect status." );
CLIDict_Def( iConnectCLIDict, "Iconnect Module Commands" ) = {
   CLIDict_Item( iconnectCmd ),
   CLIDict_Item( iconnectDbg ),
   CLIDict_Item( iconnectIdl ),
   CLIDict_Item( iconnectLst ),
   CLIDict_Item( iconnectMst ),
   CLIDict_Item( iconnectRst ),
   CLIDict_Item( iconnectSts ),
   { 0, 0, 0 } // Null entry for dictionary end
};


// --------- Messages --------------

cable_check_msg_t       ic_cable_check_msg         = { {CmdCommand_SYN, 0x01, sizeof(cable_check_msg_t), CmdCableCheck}, UARTConnectCableCheckLength_define, {0xD2}};
id_request_msg_t        ic_id_request_msg          = { {CmdCommand_SYN, 0x01, sizeof(id_request_msg_t), CmdIdRequest}, 0x00 };
id_enum_msg_t           ic_id_enum_msg             = { {CmdCommand_SYN, 0x01, sizeof(id_enum_msg_t), CmdIdEnumeration}, 0xFF};
id_report_msg_t         ic_id_report_msg           = { {CmdCommand_SYN, 0x01, sizeof(id_report_msg_t), CmdIdReport}, 0xFF};

// -- Connect send functions --

// patternLen defines how many bytes should the incrementing pattern have
void Iconnect_send_CableCheck()
{
   // Send 0xD2 (11010010) for each argument
   uint8_t value = 0xD2;
   for ( uint8_t c = 0; c < UARTConnectCableCheckLength_define; c++ )
   {
      ic_cable_check_msg.pattern[c] = value;
   }
   upipe_send_msg(connect_node.pipe[Iconnect_up], &ic_cable_check_msg.header);
   upipe_send_msg(connect_node.pipe[Iconnect_down], &ic_cable_check_msg.header);
   connect_node.cable_checks_sent[Iconnect_up]++;
   connect_node.cable_checks_sent[Iconnect_down]++;
}

void Iconnect_send_IdRequest()
{
   upipe_send_msg(connect_node.pipe[Iconnect_up], &ic_id_request_msg.header);
}

// id is the value the next slave should enumerate as
void Iconnect_send_IdEnumeration( uint8_t id )
{
   ic_id_enum_msg.id = id;
   upipe_send_msg(connect_node.pipe[Iconnect_down], &ic_id_enum_msg.header);
}

// id is the currently assigned id to the slave
void Iconnect_send_IdReport( uint8_t id )
{
   ic_id_report_msg.id = id;
   upipe_send_msg(connect_node.pipe[Iconnect_up], &ic_id_report_msg.header);
}

// -- Connect receive functions --

uint8_t Iconnect_receive_CableCheck( Iconnect_direction dir, const msg_header* hdr )
{
   const cable_check_msg_t *msg = (cable_check_msg_t*)hdr;
   for (uint8_t i = 0; i < msg->len; ++i)
   {
      if (msg->pattern[i] != 0xD2)
      {
         warn_print("Cable Fault!");

         connect_node.cable_faults[dir]++;
         connect_node.cable_ok[dir] = 0;
         return SUCCESS;
      }
   }

   connect_node.cable_checks_recv[dir]++;
   connect_node.cable_ok[dir] = 1;

   if ( Iconnect_debug )
   {
      dbug_msg("CABLECHECK RECEIVE");
   }
   return SUCCESS;

}

uint8_t Iconnect_receive_slave_CableCheck( const msg_header* hdr )
{
   return Iconnect_receive_CableCheck(Iconnect_down, hdr);
}

uint8_t Iconnect_receive_master_CableCheck( const msg_header* hdr )
{
   return Iconnect_receive_CableCheck(Iconnect_up, hdr);
}

uint8_t Iconnect_receive_slave_IdRequest( const msg_header *hdr )
{
   dbug_print("IdRequest");

   // Check if master, begin IdEnumeration
   if ( connect_node.master )
   {
      // The first device is always id 1
      // Id 0 is reserved for the master
      Iconnect_send_IdEnumeration( 1 );
   }
   // Propagate IdRequest
   else
   {
      Iconnect_send_IdRequest();
   }
   return SUCCESS;
}


uint8_t Iconnect_receive_master_IdRequest( const msg_header *hdr )
{
   dbug_print("IdRequest");
   erro_print("Invalid IdRequest direction...");
   return SUCCESS;
}

uint8_t Iconnect_receive_slave_IdEnumeration( const msg_header *hdr )
{
   dbug_print("IdEnumeration");
   erro_print("Invalid IdEnumeration direction...");
   return SUCCESS;
}

uint8_t Iconnect_receive_master_IdEnumeration( const msg_header *hdr )
{
   dbug_print("IdEnumeration");

   const id_enum_msg_t *msg = (const id_enum_msg_t*)hdr;

   // if id enum received, then node is not master
   connect_node.master = 0;

   // Set the device id
   connect_node.id = msg->id;

   // Send reponse back to master
   Iconnect_send_IdReport( msg->id );

   // Propogate next Id if the connection is ok
   if ( connect_node.cable_ok[Iconnect_down] )
   {
      Iconnect_send_IdEnumeration( msg->id + 1 );
   }

   return SUCCESS;
}

uint8_t Iconnect_receive_IdReport( const msg_header *hdr )
{
   dbug_print("IdReport");

   const id_report_msg_t *msg = (const id_report_msg_t*)hdr;

   // Track Id response if master
   if ( connect_node.master )
   {
      info_msg("Id Reported: ");
      printHex( msg->id );
      print( NL );

      return SUCCESS;
   }
   // Propagate id if yet another slave
   else
   {
      Iconnect_send_IdReport( msg->id );
   }

   return SUCCESS;
}

// Resets the state of the UART buffers and state variables
void Iconnect_reset()
{
   upipe_reset(connect_node.pipe[Iconnect_down], UART_Buffer_Size);
   upipe_reset(connect_node.pipe[Iconnect_up], UART_Buffer_Size);
}


error_code_t Iconnect_setup()
{
   CLI_registerDictionary( iConnectCLIDict, iConnectCLIDictName );
   
   connect_node.id = 0xFF;

   error_code_t err;
   err = upipe_init(&connect_node.pipe[Iconnect_down], Iconnect_down, Iconnect_baud, UART_Buffer_Size);
   if (err != SUCCESS)
   {
      erro_print("Failed to setup slave uart...");
      printHex32(err);
      return err;
   }

   err = upipe_init(&connect_node.pipe[Iconnect_up], Iconnect_up, Iconnect_baud, UART_Buffer_Size);
   if (err != SUCCESS)
   {
      erro_print("Failed to setup slave uart...");
      printHex32(err);
      return err;
   }

   //// register slave pipe callbacks
   upipe_register_callback(connect_node.pipe[Iconnect_down], CmdCableCheck,      &Iconnect_receive_slave_CableCheck);
   upipe_register_callback(connect_node.pipe[Iconnect_down], CmdIdRequest,       &Iconnect_receive_slave_IdRequest);
   upipe_register_callback(connect_node.pipe[Iconnect_down], CmdIdEnumeration,   &Iconnect_receive_slave_IdEnumeration);
   upipe_register_callback(connect_node.pipe[Iconnect_down], CmdIdReport,        &Iconnect_receive_IdReport);

   //// register master pipe callbacks
   upipe_register_callback(connect_node.pipe[Iconnect_up], CmdCableCheck,        &Iconnect_receive_master_CableCheck);
   upipe_register_callback(connect_node.pipe[Iconnect_up], CmdIdRequest,         &Iconnect_receive_master_IdRequest);
   upipe_register_callback(connect_node.pipe[Iconnect_up], CmdIdEnumeration,     &Iconnect_receive_master_IdEnumeration);
   upipe_register_callback(connect_node.pipe[Iconnect_up], CmdIdReport,          &Iconnect_receive_IdReport);

   upipe_reset(connect_node.pipe[Iconnect_down], UART_Buffer_Size);
   upipe_reset(connect_node.pipe[Iconnect_up], UART_Buffer_Size);

   //// mark node as configured
   connect_node.state = state_configured;
   info_msg("Iconnect node state: CONFIGURED" NL);

   Iconnect_reset();

   return SUCCESS;
}

error_code_t Iconnect_send_msg(Iconnect_direction dir, const msg_header *hdr)
{
   if ( connect_node.state != state_up )
      return NOT_READY;

   if ( !connect_node.cable_ok[dir] )
      return LINK_DOWN;

   return upipe_send_msg(connect_node.pipe[dir], hdr);
}

error_code_t Iconnect_send_variable_msg(Iconnect_direction dir, msg_header *hdr, const uint8_t *var, uint8_t var_size)
{
   if ( connect_node.state != state_up )
      return NOT_READY;

   if ( !connect_node.cable_ok[dir] )
      return LINK_DOWN;

   return upipe_send_variable_msg(connect_node.pipe[dir], hdr, var, var_size);
}

void Iconnect_register_callback(Iconnect_direction dir, command cmd, rx_callback_t func)
{
   upipe_register_callback(connect_node.pipe[dir], cmd, func);
}

void Iconnect_send_idle( uint8_t num )
{
   upipe_send_idle(connect_node.pipe[Iconnect_up], num);
   upipe_send_idle(connect_node.pipe[Iconnect_down], num);
}

error_code_t Iconnect_process()
{
   uint32_t current_time = systick_millis_count;
   uint32_t time_diff = current_time - Iconnect_last_time;

   switch ( connect_node.state )
   {
   case state_configured:
      // send initial cable check
      Iconnect_send_CableCheck();

      // initialize last counter
      Iconnect_last_time = systick_millis_count;

      connect_node.state = state_cable_check;
      info_msg("Iconnect node state: CABLE CHECK" NL);
      break;

   case state_cable_check:
      // send 10 cable checks 100 millis apart
      if ( connect_node.cable_checks_sent[0] < 10 )
      {
         if ( time_diff > 100 )
         {
            Iconnect_send_CableCheck();
            Iconnect_last_time = current_time;
         }
      }
      // after 1 second of cable checks, test state
      else
      {
         // if upstream ok, node is slave and needs id
         if ( connect_node.cable_ok[Iconnect_up] == 1 )
         {
            Iconnect_send_IdRequest();
            connect_node.state = state_id_recv;
            info_msg("Iconnect node state: ID RECEIVE" NL);
         }
         // if upstream no ok, set as master
         else
         {
            connect_node.id = 0;
            connect_node.master = 1;
            connect_node.state = state_up;
            info_msg("Iconnect node state: UP" NL);
         }
      }
      break;

   case state_id_recv:
      if ( connect_node.id != 0xFF )
      {
         connect_node.state = state_up;
         info_msg("Iconnect node state: UP" NL);
      }
      // resend id request every 100 milli
      else if ( time_diff > 100 )
      {
         // send another request
         Iconnect_send_IdRequest();

         // reset timer to wait again
         Iconnect_last_time = current_time;
      }
      break;

   case state_up:
      if ( time_diff > 2000 ) {
         // Send a cable check command of 2 bytes
         Iconnect_send_CableCheck();
         Iconnect_last_time = current_time;
      }
      break;
   default:
      // do nothing
      break;
   }

   // Only process commands if uarts have been configured
   if ( connect_node.state != state_reset )
   {
      // Check if Tx Buffers are empty and the Tx Ring buffers have data to send
      // This happens if there was previously nothing to send
      upipe_process(connect_node.pipe[Iconnect_down]);
      upipe_process(connect_node.pipe[Iconnect_up]);
   }
   return SUCCESS;
}

// ----- CLI Command Functions -----

void cliFunc_iconnectCmd( char* args )
{
   // Parse number from argument
   //  NOTE: Only first argument is used
   char* arg1Ptr;
   char* arg2Ptr;
   CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

   print( NL );

   switch ( numToInt( &arg1Ptr[0] ) )
   {
   case CmdCableCheck:
      Iconnect_send_CableCheck();
      break;

   case CmdIdRequest:
      Iconnect_send_IdRequest();
      break;

   case CmdIdEnumeration:
      Iconnect_send_IdEnumeration( 5 );
      break;

   case CmdIdReport:
      Iconnect_send_IdReport( 8 );
      break;

   default:
      break;
   }
}

void cliFunc_iconnectDbg( char* args )
{
   print( NL );
   info_msg("Connect Debug Mode Toggle");
   Iconnect_debug = !Iconnect_debug;
}

void cliFunc_iconnectIdl( char* args )
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

   Iconnect_send_idle( count );
}

void cliFunc_iconnectLst( char* args )
{
   const char *Command_strs[] = {
      "CableCheck",
      "IdRequest",
      "IdEnumeration",
      "IdReport",
   };

   print( NL );
   info_msg("List of UARTConnect commands");
   for ( uint8_t cmd = 0; cmd < 4; cmd++ )
   {
      print( NL );
      printInt8( cmd );
      print(" - ");
      dPrint( (char*)Command_strs[ cmd ] );
   }
}

void cliFunc_iconnectMst( char* args )
{
   // Parse number from argument
   //  NOTE: Only first argument is used
   char* arg1Ptr = 0;
   char* arg2Ptr = 0;
   CLI_argumentIsolation( args, &arg1Ptr, &arg2Ptr );

   print( NL );

   // Set override
   //Connect_override = 1;

   switch ( arg1Ptr[0] )
   {
   // Disable override
   case 'd':
   case 'D':
      //Connect_override = 0;
   case 's':
   case 'S':
      info_msg("Setting device as slave.");
      connect_node.master = 0;
      connect_node.id = 0xFF;
      break;

   case 'm':
   case 'M':
   default:
      info_msg("Setting device as master.");
      connect_node.master = 1;
      connect_node.id = 0;
      break;
   }
}

void cliFunc_iconnectRst( char* args )
{
   print( NL );
   info_msg("Resetting UARTConnect state...");
   Iconnect_reset();

   // Reset node id
   connect_node.id = 0xFF;
}

void cliFunc_iconnectSts( char* args )
{
   print( NL );
   info_msg("UARTConnect Status");
   print( NL "Node Type:\t" );
   print( connect_node.master ? "Master" : "Slave" );
   print( NL "Node Id:\t" );
   printHex( connect_node.id );
   print( NL "Node State:\t" );
   printHex( connect_node.state );
   print( NL "Upstream <=" NL "\tStatus:\t");
   printHex( connect_node.cable_ok[Iconnect_up] );
   print( NL "\tChecks:\t");
   printHex( connect_node.cable_checks_sent[Iconnect_up] );
   print( NL "\tFaults:\t");
   printHex( connect_node.cable_faults[Iconnect_up] );
   print("/");
   printHex( connect_node.cable_checks_recv[Iconnect_up] );
   print( NL "Downstream <=" NL "\tStatus:\t");
   printHex( connect_node.cable_ok[Iconnect_down] );
   print( NL "\tChecks:\t");
   printHex( connect_node.cable_checks_sent[Iconnect_down] );
   print( NL "\tFaults:\t");
   printHex( connect_node.cable_faults[Iconnect_down] );
   print("/");
   printHex( connect_node.cable_checks_recv[Iconnect_down] );
}


