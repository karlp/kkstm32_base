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
#include "systick_ms.h"  // provides delay_ms and delay_us

/**
 * For STM32, need to do it by hand, NSS isn't useful...
 */
void mrf_select(void) {
    GPIO_LOW(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
    delay_us(15);
}

void mrf_deselect(void) {
    delay_us(15);
    GPIO_HIGH(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
    delay_us(15);
}

void _delay_ms(int ms) {
    delay_ms(ms);
}
