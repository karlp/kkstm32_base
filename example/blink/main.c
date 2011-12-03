
#if CONFIG_STM32VL_DISCOVERY

#include "stm32f10x.h"
#define LED_PORT GPIOC

#define LED_BLUE (1 << 8) /* port C, pin 8 */
#define LED_GREEN (1 << 9) /* port C, pin 9 */
#define LED_ORANGE 0
#define LED_RED 0

static inline void setup_leds(void)
{
    // Make sure clocks work...
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    // Make sure we're in mode 0, with output push-pull
    LED_PORT->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0;
    LED_PORT->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9);
}

#elif CONFIG_STM32L_DISCOVERY

#include "stm32l1xx.h"
#define LED_PORT GPIOB

#define LED_BLUE (1 << 6) /* port B, pin 6 */
#define LED_GREEN (1 << 7) /* port B, pin 7 */
#define LED_ORANGE 0
#define LED_RED 0

static inline void setup_leds(void)
{
    // Make sure clocks work..
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    // set to outputs.
    LED_PORT->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
}

#elif CONFIG_STM32F4_DISCOVERY

#include "stm32f4xx.h"
#define LED_PORT GPIOD

#define LED_GREEN (1 << 12) /* port D, pin 12 */
#define LED_ORANGE (1 << 13) /* port D, pin 13 */
#define LED_RED (1 << 14) /* port D, pin 14 */
#define LED_BLUE (1 << 15) /* port D, pin 15 */

static inline void setup_leds(void)
{
    // Make sure clocks work..
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    // set to outputs.
    LED_PORT->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 |
        GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
}

#else
#error "Architecture must be defined!"
#endif /* otherwise, error */

static inline void switch_leds_on(void)
{
    LED_PORT->ODR = LED_BLUE | LED_GREEN | LED_ORANGE | LED_RED;
}

static inline void switch_leds_off(void)
{
    LED_PORT->ODR = 0;
}

#define delay()						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < 1000000; ++i)				\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)

void main(void)
{
  setup_leds();

  while (1)
  {
    switch_leds_on();
    delay();
    switch_leds_off();
    delay();
  }
}
