#include "stm32f4xx.h"                  // Device header
#include "gpio.h"
#include "spi.h"
#include "ws_leds.h"

#define tl 1
#define th 3

#define zero 0b11000000
#define one 0b11111000


void delay_nops (uint32_t dlyTicks) {
  
  for(uint32_t dly =0; dly < dlyTicks; dly++) { __NOP(); }
}

void ws_init(void) {
  
  for(uint16_t i=0; i<WS2812B_LEDS; i++)
  {
    ws_leds[i].red = 0x00;
    ws_leds[i].green = 0x00;
    ws_leds[i].blue = 0x00;
  }
}

void ws_set_led(uint16_t id, ws2812b_color color) {
    ws_leds[id].red = color.red;
    ws_leds[id].green = color.green;
    ws_leds[id].blue = color.blue;
}

void ws_spi(void) {
 
  for(uint16_t i=0, j=0; i<WS2812B_LEDS; i++)
  {
    for(int8_t k=7; k>=0; k--)
    {
      if((ws_leds[i].green & (1<<k)) == 0)
        ws_data[j] = zero;
      else
        ws_data[j] = one;
      j++;
    }

    //RED
    for(int8_t k=7; k>=0; k--)
    {
      if((ws_leds[i].red & (1<<k)) == 0)
        ws_data[j] = zero;
      else
        ws_data[j] = one;
      j++;
    }

    //BLUE
    for(int8_t k=7; k>=0; k--)
    {
      if((ws_leds[i].blue & (1<<k)) == 0)
        ws_data[j] = zero;
      else
        ws_data[j] = one;
      j++;
    }
  }
  spi_transmit(ws_data, WS2812B_LEDS*8*3);

}

void ws_spi_zero(void) {
  uint8_t ws_data[24] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  spi_transmit(ws_data, 12);
}