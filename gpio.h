#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/* LED */
#define LED_PINPOS                 5
#define LED_PORT                   GPIOA

/* WSLEDS */
#define WSLEDS_PINPOS              10
#define WSLEDS_PORT                GPIOA

#define BTN_PINPOS                 13
#define BTN_PORT                   GPIOC

typedef enum gpio_pin_e {
  GPIO_PIN_WSLEDS,
  GPIO_PIN_LED,
  GPIO_PIN_BTN,
} gpio_pin_t;

void gpio_up(gpio_pin_t pin);
void gpio_down(gpio_pin_t pin);

uint8_t gpio_get(gpio_pin_t pin);

void gpio_init(gpio_pin_t pin);

void gpio_init_input(gpio_pin_t pin);
void gpio_init_input_pu(gpio_pin_t pin);
void gpio_init_input_pd(gpio_pin_t pin);
void gpio_init_af(gpio_pin_t pin, unsigned af_no);
#endif

