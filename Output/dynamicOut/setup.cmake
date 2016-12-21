###| CMake Kiibohd Controller USB Module |###
#
# Written by Jacob Alexander in 2011-2016 for the Kiibohd Controller
#
# Released into the Public Domain
#
###

###
# Required Submodules
#

AddModule ( Output pjrcUSB )
AddModule ( Output uartFwd )

###
# Module C files
#

set ( Module_SRCS
	output_com.c
)

# Remove duplicate output_com.c files from pjrcUSB and uartOut
list ( REMOVE_ITEM Output_SRCS
	Output/pjrcUSB/output_com.c
	Output/uartFwd/output_com.c
)
###
# Compiler Family Compatibility
#
set( ModuleCompatibility
	arm
)

