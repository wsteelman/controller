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

#include_directories (
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/drivers_nrf/uart/
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/drivers_nrf/hal/
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/device/
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/toolchain/
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/toolchain/cmsis/include/
#	/Users/will/install/nRF5_SDK_12.1.0_SDK/components/libraries/util/
#   /Users/will/install/nRF5_SDK_12.1.0_SDK/components/drivers_nrf/nrf_soc_nosd/
#)

#add_definitions(
#   -DNRF52
#   -DBOARD_PCA10040
#   -DNRF52832
#   -DNRF52_PAN_64
#   -DNRF52_PAN_12
#   -DNRF52_PAN_15
#   -DNRF52_PAN_58
#   -DNRF52_PAN_55
#   -DNRF52_PAN_54
#   -DNRF52_PAN_31
#   -DNRF52_PAN_30
#   -DNRF52_PAN_51
#   -DNRF52_PAN_36
#   -DNRF52_PAN_53
#   -DS132
#   -DCONFIG_GPIO_AS_PINRESET
#   -DBLE_STACK_SUPPORT_REQD
#   -DNRF_SD_BLE_API_VERSION=3
#   -DSWI_DISABLE0
#   -DNRF52_PAN_20
#   -DSOFTDEVICE_PRESENT
#   -DNRF52_PAN_62
#   -DNRF52_PAN_63
#)

###
# Compiler Family Compatibility
#
set ( ModuleCompatibility
	arm
)

