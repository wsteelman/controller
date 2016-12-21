/* Copyright (C) 2011-2016 by Jacob Alexander
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// ----- Includes -----

// Compiler Includes
#include <Lib/OutputLib.h>

// Project Includes
#include <cli.h>
#include <print.h>
#include <interconnect.h>

// KLL
#include <kll_defs.h>

// Local Includes
#include "uart_output_com.h"

remote_capability_msg_t output_remote_capability_msg   = { {CmdCommand_SYN, 0x01, sizeof(remote_capability_msg_t), CmdRemoteCapability}, 0xFF, 0, 0, 0, 0};

// ----- Variables -----
uint8_t keys_sent = 0;

// ----- Capabilities -----

void send_remote_capability( uint8_t index, uint8_t state, uint8_t stateType, uint8_t *args )
{
   // Prepare header
   output_remote_capability_msg.id = 1;  // TODO: don't hardcode this
   output_remote_capability_msg.capability_index = index;
   output_remote_capability_msg.state = state;
   output_remote_capability_msg.state_type = stateType;

   // get argument count
   extern const Capability CapabilitiesList[]; // See generatedKeymap.h
   uint8_t num_args = CapabilitiesList[ index ].argCount;
   output_remote_capability_msg.num_args = num_args;

   // send message
   Iconnect_send_variable_msg(Iconnect_up, &output_remote_capability_msg.header, args, num_args);

} 

// Set Boot Keyboard Protocol
void uart_Output_kbdProtocolBoot_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_kbdProtocolBoot_capability_index, state, stateType, args);
#endif
}


// Set NKRO Keyboard Protocol
void uart_Output_kbdProtocolNKRO_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_kbdProtocolNKRO_capability_index, state, stateType, args);
#endif
}


// Toggle Keyboard Protocol
void uart_Output_toggleKbdProtocol_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_toggleKbdProtocol_capability_index, state, stateType, args);
#endif
}


// Sends a Consumer Control code to the USB Output buffer
void uart_Output_consCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_consCtrlSend_capability_index, state, stateType, args);
#endif
}


// Ignores the given key status update
// Used to prevent fall-through, this is the None keyword in KLL
void uart_Output_noneSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   send_remote_capability(Output_noneSend_capability_index, state, stateType, args);
}


// Sends a System Control code to the USB Output buffer
void uart_Output_sysCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_sysCtrlSend_capability_index, state, stateType, args);
#endif
}


// Adds a single USB Code to the USB Output buffer
// Argument #1: USB Code
void uart_Output_usbCodeSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
#if enableKeyboard_define == 1
   send_remote_capability(Output_usbCodeSend_capability_index, state, stateType, args);
   keys_sent++;
#endif
}

void uart_Output_flashMode_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   send_remote_capability(Output_flashMode_capability_index, state, stateType, args);
}

#if enableMouse_define == 1
// Sends a mouse command over the USB Output buffer
// XXX This function *will* be changing in the future
//     If you use it, be prepared that your .kll files will break in the future (post KLL 0.5)
// Argument #1: USB Mouse Button (16 bit)
// Argument #2: USB X Axis (16 bit) relative
// Argument #3: USB Y Axis (16 bit) relative
void uart_Output_usbMouse_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   send_remote_capability(Output_usbMouse_capability_index, state, stateType, args);
}
#endif


// ----- Functions -----

inline uint8_t uart_Output_available()
{
   return Iconnect_available(Iconnect_up);
}

inline uint8_t uart_Output_keys_sent()
{
   return keys_sent;
}

inline void uart_Output_reset_buffers()
{
   keys_sent = 0;
}


// Flush Key buffers
void uart_Output_flushBuffers()
{
}


// USB Module Setup
inline void uart_Output_setup()
{

}


// USB Data Send
inline void uart_Output_send()
{
   keys_sent = 0;
}


// Sets the device into firmware reload mode
inline void uart_Output_firmwareReload()
{
}


// USB Input buffer available
inline unsigned int uart_Output_availablechar()
{
   return 0;
}


// USB Get Character from input buffer
inline int uart_Output_getchar()
{
   return 0;
}


// USB Send Character to output buffer
inline int uart_Output_putchar( char c )
{
   return 0;
}


// USB Send String to output buffer, null terminated
inline int uart_Output_putstr( char* str )
{
   return 0;
}


// Soft Chip Reset
inline void uart_Output_softReset()
{
}


// USB RawIO buffer available
inline unsigned int uart_Output_rawio_availablechar()
{
   return 0;
}


// USB RawIO get buffer
// XXX Must be a 64 byte buffer
inline int uart_Output_rawio_getbuffer( char* buffer )
{
   return 0;
}


// USB RawIO send buffer
// XXX Must be a 64 byte buffer
inline int uart_Output_rawio_sendbuffer( char* buffer )
{
   return 0;
}


// Update USB current (mA)
// Triggers power change event
void uart_Output_update_usb_current( unsigned int current )
{
}


// Update external current (mA)
// Triggers power change event
void uart_Output_update_external_current( unsigned int current )
{
}


// Power/Current Available
unsigned int uart_Output_current_available()
{
   return 0;
}
