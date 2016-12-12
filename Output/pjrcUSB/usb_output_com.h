/* Copyright (C) 2013-2016 by Jacob Alexander
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

#pragma once

// ----- Includes -----

// Compiler Includes
#include <stdint.h>

// Local Includes
#include <buildvars.h> // Defines USB Parameters, partially generated by CMake



// ----- Defines -----

// Max size of key buffer needed for NKRO
// Boot mode uses only the first 6 bytes
#define USB_NKRO_BITFIELD_SIZE_KEYS 27
#define USB_BOOT_MAX_KEYS 6



// ----- Enumerations -----

// USB NKRO state transitions (indicates which Report ID's need refreshing)
// Boot mode just checks if any keys were changed (as everything is sent every time)
typedef enum USBKeyChangeState {
	USBKeyChangeState_None          = 0x00,
	USBKeyChangeState_Modifiers     = 0x01,
	USBKeyChangeState_MainKeys      = 0x02,
	USBKeyChangeState_SecondaryKeys = 0x04,
	USBKeyChangeState_TertiaryKeys  = 0x08,
	USBKeyChangeState_QuartiaryKeys = 0x10,
	USBKeyChangeState_System        = 0x20,
	USBKeyChangeState_Consumer      = 0x40,
	USBKeyChangeState_All           = 0x7F,
} USBKeyChangeState;

// Allows for selective USB descriptor pushes
// However, in most cases everything is updated for each packet push
typedef enum USBMouseChangeState {
	USBMouseChangeState_None     = 0x00,
	USBMouseChangeState_Buttons  = 0x01,
	USBMouseChangeState_Relative = 0x02,
	USBMouseChangeState_All      = 0x03,
} USBMouseChangeState;



// ----- Variables -----

// Variables used to communciate to the output module
// XXX Even if the output module is not USB, this is internally understood keymapping scheme
extern          uint8_t  USBKeys_Modifiers;
extern          uint8_t  USBKeys_Keys[USB_NKRO_BITFIELD_SIZE_KEYS];
extern          uint8_t  USBKeys_Sent;
extern volatile uint8_t  USBKeys_LEDs;

extern          uint8_t  USBKeys_SysCtrl;  // 1KRO container for System Control HID table
extern          uint16_t USBKeys_ConsCtrl; // 1KRO container for Consumer Control HID table

extern volatile uint8_t  USBKeys_Protocol; // 0 - Boot Mode, 1 - NKRO Mode

extern volatile uint16_t USBMouse_Buttons; // Bitmask for mouse buttons
extern volatile uint16_t USBMouse_Relative_x;
extern volatile uint16_t USBMouse_Relative_y;

// Keeps track of the idle timeout refresh (used on Mac OSX)
extern          uint8_t  USBKeys_Idle_Config;
extern          uint32_t USBKeys_Idle_Expiry;
extern          uint8_t  USBKeys_Idle_Count; // AVR only

extern USBKeyChangeState   USBKeys_Changed;
extern USBMouseChangeState USBMouse_Changed;

extern volatile uint8_t  Output_Available; // 0 - Output module not fully functional, 1 - Output module working

extern          uint8_t  Output_DebugMode; // 0 - Debug disabled, 1 - Debug enabled

extern          uint16_t Output_ExtCurrent_Available; // mA - Set by outside module if not using USB (i.e. Interconnect)

extern volatile uint32_t USBInit_TimeStart; // Timetamp when usb_init was triggered
extern volatile uint32_t USBInit_TimeEnd;   // Timetamp since last call to the Configuration endpoint
extern volatile uint16_t USBInit_Ticks;     // Number of times the end time has been updated



// ----- Functions -----

void usb_Output_setup();
void usb_Output_send();
void usb_Output_flushBuffers();

uint8_t usb_Output_available();
uint8_t usb_Output_keys_sent();
void usb_Output_reset_buffers();


void usb_Output_firmwareReload();
void usb_Output_softReset();

// Relies on USB serial module
unsigned int usb_Output_availablechar();

// Returns the total mA available (total, if used in a chain, each device will have to use a slice of it)
unsigned int usb_Output_current_available();

void usb_Output_update_external_current( unsigned int current );
void usb_Output_update_usb_current( unsigned int current );

int usb_Output_getchar();
int usb_Output_putchar( char c );
int usb_Output_putstr( char* str );

// ----- Capabilities -----

// Set Boot Keyboard Protocol
void usb_Output_kbdProtocolBoot_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Set NKRO Keyboard Protocol
void usb_Output_kbdProtocolNKRO_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Toggle Keyboard Protocol
void usb_Output_toggleKbdProtocol_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Sends a Consumer Control code to the USB Output buffer
void usb_Output_consCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Ignores the given key status update
// Used to prevent fall-through, this is the None keyword in KLL
void usb_Output_noneSend_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Sends a System Control code to the USB Output buffer
void usb_Output_sysCtrlSend_capability( uint8_t state, uint8_t stateType, uint8_t *args );

// Adds a single USB Code to the USB Output buffer
// Argument #1: USB Code
void usb_Output_usbCodeSend_capability( uint8_t state, uint8_t stateType, uint8_t *args );

void usb_Output_flashMode_capability( uint8_t state, uint8_t stateType, uint8_t *args );

#if enableMouse_define == 1
// Sends a mouse command over the USB Output buffer
// XXX This function *will* be changing in the future
//     If you use it, be prepared that your .kll files will break in the future (post KLL 0.5)
// Argument #1: USB Mouse Button (16 bit)
// Argument #2: USB X Axis (16 bit) relative
// Argument #3: USB Y Axis (16 bit) relative
void usb_Output_usbMouse_capability( uint8_t state, uint8_t stateType, uint8_t *args );
#endif




