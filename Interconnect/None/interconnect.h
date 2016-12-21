#pragma once

#include <error.h>

error_code_t Iconnect_setup();

error_code_t Iconnect_process();

void Iconnect_reset();

inline uint8_t Iconnect_node_id()
{
   return 0xFF;
}
