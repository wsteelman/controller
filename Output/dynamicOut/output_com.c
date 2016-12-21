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
#include <uart_output_com.h>
#include <usb_output_com.h>
#include <interconnect.h>

typedef struct output_funcs_t
{
   void           (*setup)();
   void           (*send)();
   void           (*flush_buffers)();
   uint8_t        (*available)();
   uint8_t        (*keys_sent)();
   void           (*reset_buffers)();
   void           (*firmware_reload)();
   void           (*soft_reset)();
   unsigned int   (*availablechar)();
   unsigned int   (*current_available)();
   void           (*update_external_current)(unsigned int current);
   void           (*update_usb_current)(unsigned int current);
   int            (*getchar)();
   int            (*putchar)(char c);
   int            (*putstr)(char *str);

   void (*kbdProtocolBoot_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*kbdProtocolNKRO_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*toggleKbdProtocol_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*consCtrlSend_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*noneSend_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*sysCtrlSend_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*usbCodeSend_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*flashMode_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
   void (*usbMouse_capability)(uint8_t state, uint8_t stateType, uint8_t *args);
} output_funcs_t;

#define usb_funcs {                          \
   &usb_Output_setup,                        \
   &usb_Output_send,                         \
   &usb_Output_flushBuffers,                 \
   &usb_Output_available,                    \
   &usb_Output_keys_sent,                    \
   &usb_Output_reset_buffers,                \
   &usb_Output_firmwareReload,               \
   &usb_Output_softReset,                    \
   &usb_Output_availablechar,                \
   &usb_Output_current_available,            \
   &usb_Output_update_external_current,      \
   &usb_Output_update_usb_current,           \
   &usb_Output_getchar,                      \
   &usb_Output_putchar,                      \
   &usb_Output_putstr,                       \
   &usb_Output_kbdProtocolBoot_capability,   \
   &usb_Output_kbdProtocolNKRO_capability,   \
   &usb_Output_toggleKbdProtocol_capability, \
   &usb_Output_consCtrlSend_capability,      \
   &usb_Output_noneSend_capability,          \
   &usb_Output_sysCtrlSend_capability,       \
   &usb_Output_usbCodeSend_capability,       \
   &usb_Output_flashMode_capability,         \
   &usb_Output_usbMouse_capability           \
}

#define uart_funcs {                          \
   &uart_Output_setup,                        \
   &uart_Output_send,                         \
   &uart_Output_flushBuffers,                 \
   &uart_Output_available,                    \
   &uart_Output_keys_sent,                    \
   &uart_Output_reset_buffers,                \
   &uart_Output_firmwareReload,               \
   &uart_Output_softReset,                    \
   &uart_Output_availablechar,                \
   &uart_Output_current_available,            \
   &uart_Output_update_external_current,      \
   &uart_Output_update_usb_current,           \
   &uart_Output_getchar,                      \
   &uart_Output_putchar,                      \
   &uart_Output_putstr,                       \
   &uart_Output_kbdProtocolBoot_capability,   \
   &uart_Output_kbdProtocolNKRO_capability,   \
   &uart_Output_toggleKbdProtocol_capability, \
   &uart_Output_consCtrlSend_capability,      \
   &uart_Output_noneSend_capability,          \
   &uart_Output_sysCtrlSend_capability,       \
   &uart_Output_usbCodeSend_capability,       \
   &uart_Output_flashMode_capability,         \
   &uart_Output_usbMouse_capability           \
}

output_funcs_t output_funcs[2] = {usb_funcs, uart_funcs};

// ----- Variables -----
protocol_t protocol = usb_protocol;

// ----- Capabilities -----

// Set Boot Keyboard Protocol
void Output_kbdProtocolBoot_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].kbdProtocolBoot_capability(state, stateType, args);
}


// Set NKRO Keyboard Protocol
void Output_kbdProtocolNKRO_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].kbdProtocolNKRO_capability(state, stateType, args);
}


// Toggle Keyboard Protocol
void Output_toggleKbdProtocol_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].toggleKbdProtocol_capability(state, stateType, args);
}


// Sends a Consumer Control code to the USB Output buffer
void Output_consCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].consCtrlSend_capability(state, stateType, args);
}


// Ignores the given key status update
// Used to prevent fall-through, this is the None keyword in KLL
void Output_noneSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].noneSend_capability(state, stateType, args);
}


// Sends a System Control code to the USB Output buffer
void Output_sysCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].sysCtrlSend_capability(state, stateType, args);
}


// Adds a single USB Code to the USB Output buffer
// Argument #1: USB Code
void Output_usbCodeSend_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].usbCodeSend_capability(state, stateType, args);
}

void Output_flashMode_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
   output_funcs[protocol].flashMode_capability(state, stateType, args);
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
   output_funcs[protocol].usbMouse_capability(state, stateType, args);
}
#endif



// ----- Functions -----

inline uint8_t Output_available()
{
   return output_funcs[protocol].available();
}

inline uint8_t Output_keys_sent()
{
   return output_funcs[protocol].keys_sent();
}

inline void Output_reset_buffers()
{
   output_funcs[protocol].reset_buffers();
}

// Flush Key buffers
inline void Output_flushBuffers()
{
   output_funcs[protocol].flush_buffers();
}

// USB Module Setup
inline void Output_setup()
{
   for (uint8_t i = 0; i < protocol_count; ++i)
   {
      output_funcs[i].setup();
   }
}

// USB Data Send
inline void Output_send()
{
   output_funcs[protocol].send();
}

// Sets the device into firmware reload mode
inline void Output_firmwareReload()
{
   output_funcs[protocol].firmware_reload();
}

// USB Input buffer available
inline unsigned int Output_availablechar()
{
   return output_funcs[protocol].availablechar();
}

// USB Get Character from input buffer
inline int Output_getchar()
{
   return output_funcs[protocol].getchar();
}

// USB Send Character to output buffer
inline int Output_putchar( char c )
{
   return output_funcs[protocol].putchar(c);
}

// USB Send String to output buffer, null terminated
inline int Output_putstr( char* str )
{
   return output_funcs[protocol].putstr(str);
}

// Soft Chip Reset
inline void Output_softReset()
{
   output_funcs[protocol].soft_reset();
}

// Update USB current (mA)
// Triggers power change event
void Output_update_usb_current( unsigned int current )
{
   output_funcs[protocol].update_usb_current(current);
}

// Update external current (mA)
// Triggers power change event
void Output_update_external_current( unsigned int current )
{
   output_funcs[protocol].update_external_current(current);
}

// Power/Current Available
unsigned int Output_current_available()
{
   return output_funcs[protocol].current_available();
}

