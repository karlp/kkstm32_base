/**
 * Basic code for providing routines based on a millisecond systick timer
 * 
 */

#include <stdint.h>

#include "stm32l1xx_rcc.h"

static volatile int64_t ksystick;
static uint32_t clocks_per_us;

/**
 * Keep track of a current milliseconds since boot ticker
 * @return 
 */
int64_t millis(void)
{
    return ksystick;
}

void _delay_loop(int count) {
    volatile int counter = count;
    while (counter--);
}

/**
 * Busy loop in uS.  It will be off by a fair bit....
 * @param us
 */
void delay_us(int us) {
    _delay_loop(us * clocks_per_us);
}

/**
 * Busy loop for X ms USES INTERRUPTS
 * @param ms
 */
void delay_ms(int ms) {
    int64_t now = millis();
    while (millis() - ms < now) {
        ;
    }
}

void systick_ms_init(void) {
    ksystick = 0;
    RCC_ClocksTypeDef clockinfo;
    RCC_GetClocksFreq(&clockinfo);
    // regardless of clock speed this gives us 1000 ticks per second
    SysTick_Config(clockinfo.SYSCLK_Frequency / 1000);
    clocks_per_us = clockinfo.SYSCLK_Frequency / 1000000;
}


void SysTick_Handler(void)
{
    ksystick++;
}


