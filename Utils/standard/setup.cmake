###| CMake Kiibohd Controller Scan Module |###
#
# Written by Jacob Alexander in 2014-2015 for the Kiibohd Controller
#
# Released into the Public Domain
#
###


###
# Module C files
#
set ( Module_SRCS
   uart.c
   dma.c
   uart_message_pipe.c
)


###
# Compiler Family Compatibility
#
set ( ModuleCompatibility
	arm
)

