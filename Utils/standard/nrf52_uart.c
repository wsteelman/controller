
#if defined (_nRF52832_)

#include "nrf52_uart2.h"
uint8_t pin_value = 0;
uart_handle uart_handles[UART_Count] = {0};

void UART0_IRQHandler(void)
{
   uart_handle *h = &uart_handles[0];
   
   if (nrf_uarte_event_check(h->dev, NRF_UARTE_EVENT_ERROR))
   {
      // TODO
   }
   else if (nrf_uarte_event_check(h->dev, NRF_UARTE_EVENT_ENDRX))
   {
      nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_ENDRX);
       
      // setup area for next dma transfer
      h->rx_idx = h->rx_next_idx;
      nrf_uarte_rx_buffer_set(h->dev, next_dma_buffer(h), DOUBLE_BUF_SIZE);

      //pin_value = ~pin_value; 
      //nrf_gpio_pin_write(7, pin_value); 
   }
   
   if (nrf_uarte_event_check(h->dev, NRF_UARTE_EVENT_RXTO))
   {
      // TODO
   }
   

   //if (nrf_uarte_event_check(h->dev, NRF_UARTE_EVENT_TXSTARTED))
   //{
   //   nrf_gpio_pin_write(7, 0); 
   //}

   if (nrf_uarte_event_check(h->dev, NRF_UARTE_EVENT_ENDTX))
   {
      nrf_uarte_event_clear(h->dev, NRF_UARTE_EVENT_ENDTX);
      h->tx_pending = 0;
   }
}

#endif 
