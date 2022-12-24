#ifndef WS_LEDS_H
#define WS_LEDS_H

#include <stdint.h>

#define WS2812B_LEDS 7*7

typedef struct ws2812b_color {
  uint8_t red, green, blue;
} ws2812b_color;

void ws_set_led(uint16_t id, ws2812b_color color);
static ws2812b_color ws_leds[WS2812B_LEDS];
static uint8_t ws_data[WS2812B_LEDS*3*8];

void delay_nops (uint32_t dlyTicks);
void ws_init(void);
void ws_spi(void);
void ws_spi_zero(void);
#endif

