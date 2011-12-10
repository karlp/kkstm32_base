/**
 * Basic code for providing routines based on a millisecond systick timer
 * 
 */

#include <stdint.h>

#include "stm32l1xx_rcc.h"

static volatile uint64_t ksystick;

/**
 * Keep track of a current milliseconds since boot ticker
 * @return 
 */
uint64_t millis(void)
{
    return ksystick;
}

/**
 * Busy loop for X ms
 * @param ms
 */
void delay_ms(int ms) {
    uint64_t now = millis();
    while (millis() - ms < now) {
        ;
    }
}

void systick_ms_init(void) {
    RCC_ClocksTypeDef clockinfo;
    RCC_GetClocksFreq(&clockinfo);
    // regardless of clock speed this gives us 1000 ticks per second
    SysTick_Config(clockinfo.SYSCLK_Frequency / 1000);
}


void SysTick_Handler(void)
{
    ksystick++;
}


