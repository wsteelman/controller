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

#include <kll_defs.h>
#include "output_com.h"
#include "uart_output_com.h"

// ----- Capabilities -----

// Set Boot Keyboard Protocol
void Output_kbdProtocolBoot_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_kbdProtocolBoot_capability( state, stateType, args );
}


// Set NKRO Keyboard Protocol
void Output_kbdProtocolNKRO_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_kbdProtocolNKRO_capability( state, stateType, args );
}


// Toggle Keyboard Protocol
void Output_toggleKbdProtocol_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_toggleKbdProtocol_capability( state, stateType, args );
}


// Sends a Consumer Control code to the USB Output buffer
void Output_consCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_consCtrlSend_capability( state, stateType, args );
}


// Ignores the given key status update
// Used to prevent fall-through, this is the None keyword in KLL
void Output_noneSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_noneSend_capability( state, stateType, args );
}


// Sends a System Control code to the USB Output buffer
void Output_sysCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_sysCtrlSend_capability( state, stateType, args );
}


// Adds a single USB Code to the USB Output buffer
// Argument #1: USB Code
void Output_usbCodeSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_usbCodeSend_capability( state, stateType, args );
}

void Output_flashMode_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_flashMode_capability( state, stateType, args );
}

#if enableMouse_define == 1
// Sends a mouse command over the USB Output buffer
// XXX This function *will* be changing in the future
//     If you use it, be prepared that your .kll files will break in the future (post KLL 0.5)
// Argument #1: USB Mouse Button (16 bit)
// Argument #2: USB X Axis (16 bit) relative
// Argument #3: USB Y Axis (16 bit) relative
void Output_usbMouse_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   uart_Output_usbMouse_capability( state, stateType, args );
}
#endif



// ----- Functions -----

inline uint8_t Output_available()
{
   return uart_Output_available();
}

inline uint8_t Output_keys_sent()
{
   return uart_Output_keys_sent();
}

inline void Output_reset_buffers()
{
   return uart_Output_reset_buffers();
}

// Flush Key buffers
inline void Output_flushBuffers()
{
   uart_Output_flushBuffers();
}

// USB Module Setup
inline void Output_setup()
{
   uart_Output_setup();
}

// USB Data Send
inline void Output_send()
{
   uart_Output_send();
}

// Sets the device into firmware reload mode
inline void Output_firmwareReload()
{
   uart_Output_firmwareReload();
}

// USB Input buffer available
inline unsigned int Output_availablechar()
{
   return uart_Output_availablechar();
}

// USB Get Character from input buffer
inline int Output_getchar()
{
   return uart_Output_getchar();
}

// USB Send Character to output buffer
inline int Output_putchar( char c )
{
   return uart_Output_putchar( c );
}

// USB Send String to output buffer, null terminated
inline int Output_putstr( char* str )
{
   return uart_Output_putstr( str );
}

// Soft Chip Reset
inline void Output_softReset()
{
   uart_Output_softReset();
}

// Update USB current (mA)
// Triggers power change event
void Output_update_usb_current( unsigned int current )
{
   uart_Output_update_usb_current( current );
}

// Update external current (mA)
// Triggers power change event
void Output_update_external_current( unsigned int current )
{
   uart_Output_update_external_current( current );
}

// Power/Current Available
unsigned int Output_current_available()
{
   return uart_Output_current_available();
}

