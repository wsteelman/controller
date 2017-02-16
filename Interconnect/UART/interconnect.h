#pragma once

#include <stdint.h>
#include "msg.h"
#include "uart_message_pipe.h"

#define PIPE_COUNT 2 2

#define ICONNECT_ENABLE_DOWN 0x01
#define ICONNECT_ENABLE_UP 0x02
#define ICONNECT_ENABLE_UP_AND_DOWN 0x03

typedef enum Iconnect_direction
{
   Iconnect_down   = 0,
   Iconnect_up     = 1,
   Iconnect_count  = 2,
} Iconnect_direction;

typedef enum Iconnect_state
{
   state_reset             = 0,
   state_configured        = 1,
   state_cable_check       = 2,
   state_id_recv           = 3,
   state_up                = 4,
} Iconnect_state;

typedef struct Iconnect_node_t
{
   uint8_t enable_mask;
   uint8_t id;
   uint8_t master;
   Iconnect_state state;
   uart_message_pipe_t *pipe[Iconnect_count];
   uint32_t cable_faults[Iconnect_count];
   uint32_t cable_checks_sent[Iconnect_count];
   uint32_t cable_checks_recv[Iconnect_count];
   uint32_t cable_ok[Iconnect_count];
} Iconnect_node_t;

extern Iconnect_node_t connect_node;

error_code_t Iconnect_setup(uint8_t enable_mask);

error_code_t Iconnect_process();

error_code_t Iconnect_send_msg(Iconnect_direction dir, const msg_header *hdr);

error_code_t Iconnect_send_variable_msg(Iconnect_direction dir, msg_header *hdr, const uint8_t *var, uint8_t var_size);

void Iconnect_register_callback(Iconnect_direction dir, command cmd, rx_callback_t func);

void Iconnect_reset();

inline uint8_t Iconnect_node_id()
{
   return connect_node.id;
}

inline uint8_t Iconnect_available(Iconnect_direction dir)
{
   return connect_node.state == state_up && connect_node.cable_ok[dir];
}
