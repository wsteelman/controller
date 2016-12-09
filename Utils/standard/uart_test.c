#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define _mk20dx256vlh7_
#include "test_regs.h"

#include "uart.h"
#include "dma.h"


int main(int argc, char **argv)
{
   printf("testing uart\n");
   printf("uart0 rx pin addr %p\n", uarts[0].rx_pin);

   printf("csr %p vs %p\n", &DMA_TCD0_CSR, dma_chnls[0].chnl_csr);
   printf("channel id %u\n", dma_next_chnl);
   return 0;
}
