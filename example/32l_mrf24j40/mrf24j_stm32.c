/* 
 * File:   mrf24j_stm32.c
 * Author: karlp
 *
 * Provides the driver (spi tx and pin toggling) required for the MRF24J40 code
 * 
 * Created on December 9, 2011, 10:46 PM
 */
#include "stdint.h"
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_spi.h"

#include "32l_mrf24j40_conf.h"
#include "lib_mrf24j.h"
#include "systick_ms.h"  // somethign that can provide delay_ms and delay_us

/**
 * For STM32, need to do it by hand, NSS isn't useful...
 */
void mrf_select(void) {
    GPIO_LOW(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
    _delay_ms(1);
}

void mrf_deselect(void) {
    _delay_ms(1);
    GPIO_HIGH(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
    _delay_ms(1);
}

    
void mrf_reset(void) {
    GPIO_LOW(GPIOB, PIN_MRF_RESET);
    _delay_ms(10);  // gut feeling
    GPIO_HIGH(GPIOB, PIN_MRF_RESET);
    _delay_ms(20);  // from manual
}

/**
 * Cannot use interrupts, will be called in interrupt context!
 * @param ms
 */
void _delay_ms(int ms) {
    //delay_ms(ms);
    delay_us(50 * ms);
}
