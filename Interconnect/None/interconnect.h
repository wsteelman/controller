#pragma once

#include <error.h>

error_code_t Iconnect_setup();

//error_code_t Iconnect_send_msg(Iconnect_direction dir, const msg_header *hdr);
 
//error_code_t Iconnect_send_variable_msg(Iconnect_direction dir, msg_header *hdr, const uint8_t *var, uint8_t var_size);

error_code_t Iconnect_process();

//void Iconnect_register_callback(Iconnect_direction dir, command cmd, rx_callback_t func);

void Iconnect_reset();
