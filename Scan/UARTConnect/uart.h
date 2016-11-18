#ifndef UART_H__
#define UART_H__

#include "error.h"
#include "dma.h"

#define UART_MAX_INDEX 3
//#define uart_buffer_size UARTConnectBufSize_define

typedef struct uart_addrs
{
   volatile uint32_t *  rx_pin;
   volatile uint32_t *  tx_pin;
   volatile uint8_t  *  bdh;              // baud rate high
   volatile uint8_t  *  bdl;              // baud rate low
   volatile uint8_t  *  c1;               // control 1
   volatile uint8_t  *  c2;               // control 2
   volatile uint8_t  *  c3;               // control 3
   volatile uint8_t  *  c4;               // control 4
   volatile uint8_t  *  c5;               // control 5
   volatile uint8_t  *  s1;               // status 1
   volatile uint8_t  *  s2;               // status 2
   volatile uint8_t  *  data;             // data
   volatile uint8_t  *  pfifo;            // fifo parameters
   volatile uint8_t  *  cfifo;            // fifo control
   volatile uint8_t  *  sfifo;            // fifo status
   volatile uint8_t  *  tcfifo;           // fifo transmit count
   uint8_t              pin_mux_mode;     // pin mux setting
   uint32_t             clk_gate_mask;    // clock gate mask for SIM_SCGC4
   uint8_t              dma_mux_rx_mask;  // mask for DMA mux
   uint8_t              irq_status_mask;  // mask for IRQ status
} uart_addrs;

typedef struct uart_handle
{
   uint8_t     id;
   uint8_t     dma_rx_chnl;
   uint8_t     tx_fifo_size;
} uart_handle;

typedef const void *uart_handle_t;

#if defined (_mk20dx256vlh7_)
   #define uart0 {                                 \
      .rx_pin           = &PORTA_PCR1,             \
      .tx_pin           = &PORTA_PCR2,             \
      .bdh              = &UART0_BDH,              \
      .bdl              = &UART0_BDL,              \
      .c1               = &UART0_C1,               \
      .c2               = &UART0_C2,               \
      .c3               = &UART0_C3,               \
      .c4               = &UART0_C4,               \
      .c5               = &UART0_C5,               \
      .s1               = &UART0_S1,               \
      .s2               = &UART0_S2,               \
      .data             = &UART0_D,                \
      .pfifo            = &UART0_PFIFO,            \
      .cfifo            = &UART0_CFIFO,            \
      .sfifo            = &UART0_SFIFO,            \
      .sfifo            = &UART0_TCFIFO,           \
      .pin_mux_mode     = 2,                       \
      .clk_gate_mask    = SIM_SCGC4_UART0,         \
      .dma_mux_rx_mask  = DMAMUX_SOURCE_UART0_RX,  \
      .irq_status_mask  = IRQ_UART0_STATUS         \
   }

   #define uart1 {                                 \
      .rx_pin           = &PORTE_PCR0,             \
      .tx_pin           = &PORTE_PCR1,             \
      .bdh              = &UART1_BDH,              \
      .bdl              = &UART1_BDL,              \
      .c1               = &UART1_C1,               \
      .c2               = &UART1_C2,               \
      .c3               = &UART1_C3,               \
      .c4               = &UART1_C4,               \
      .c5               = &UART1_C5,               \
      .s1               = &UART1_S1,               \
      .s2               = &UART1_S2,               \
      .data             = &UART1_D,                \
      .pfifo            = &UART1_PFIFO,            \
      .cfifo            = &UART1_CFIFO,            \
      .sfifo            = &UART1_SFIFO,            \
      .sfifo            = &UART1_TCFIFO,           \
      .pin_mux_mode     = 3,                       \
      .clk_gate_mask    = SIM_SCGC4_UART1,         \
      .dma_mux_rx_mask  = DMAMUX_SOURCE_UART1_RX,  \
      .irq_status_mask  = IRQ_UART1_STATUS         \
   }

   uart_addrs uarts[2] = {uart0, uart1};
#endif

uart_handle uart_handles[UART_MAX_INDEX+1];

uart_handle_t uart_config (uint8_t uart_id, uint16_t baud)
{
   if (uart_id > UART_MAX_INDEX)
      return NULL;

   const uart_addrs *inf = &uarts[uart_id];
   uart_handle *handle = &uart_handles[uart_id];
   handle->id = uart_id;
   handle->dma_rx_chnl = 0xFF;

   // Turn on the clock
   SIM_SCGC4 |= inf->clk_gate_mask; // Disable clock gating

   // pin setup
   *(inf->rx_pin) = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE |
                    PORT_PCR_MUX(inf->pin_mux_mode);
   *(inf->tx_pin) = PORT_PCR_DSE | PORT_PCR_SRE |
                    PORT_PCR_MUX(inf->pin_mux_mode);

   // baud rate
   *(inf->bdh) = (uint8_t)(baud >> 8);
   *(inf->bdl) = (uint8_t)baud;
   *(inf->c4) = 0;

   // 8 bit, Even Parity, Idle Character bit after stop
   // NOTE: For 8 bit with Parity you must enable 9 bit transmission (pg. 1065)
   //       You only need to use UART0_D for 8 bit reading/writing though
   // UART_C1_M UART_C1_PE UART_C1_PT UART_C1_ILT
   *(inf->c1) = UART_C1_M | UART_C1_PE | UART_C1_ILT;

   // Only using Tx Fifos
   *(inf->pfifo) = UART_PFIFO_TXFE;
   handle->tx_fifo_size = (*(inf->pfifo) & UART_PFIFO_TXFIFOSIZE) >> 2;

   return (void*) handle;
}

error_code_t uart_rx_dma_setup(uint8_t chnl, uart_handle_t handle, uint32_t *dst, uint32_t buf_size)
{
   uart_handle *h = (uart_handle*)handle;
   const uart_addrs *inf = &uarts[h->id];

   if (h->dma_rx_chnl != 0xFF)
      return DMA_CHANNEL_IN_USE;
   h->dma_rx_chnl = chnl;

   error_code_t err = dma_config(chnl, inf->dma_mux_rx_mask, (uint32_t*)inf->data, dst, buf_size);
   if (err != DMA_SUCCESS)
      return err;

   // enable DMA requests on receive(requires RX interrupts)
   *inf->c5 = UART_C5_RDMAS;

   // Add interrupts to the vector table
   NVIC_ENABLE_IRQ( inf->irq_status_mask );

   return SUCCESS;
}

uint8_t uart_rx_dma_buffer_position(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return dma_buffer_position(h->dma_rx_chnl);
}

void uart_start(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].c2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
}

void uart_stop(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].c2 = 0;
}

uint8_t uart_tx_fifo_size(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return h->tx_fifo_size;
}

uint8_t uart_tx_fifo_pending(uart_handle_t handle)
{
   uart_handle *h = (uart_handle*)handle;
   return *uarts[h->id].tcfifo;
}

void uart_send(uart_handle_t handle, uint8_t data)
{
   uart_handle *h = (uart_handle*)handle;
   *uarts[h->id].data = data;
}

#endif  // #ifndef UART_H__
