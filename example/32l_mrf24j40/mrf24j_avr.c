/**
 * Karl Palsson,
 * AVR specific portions of the MRF24J40 driver...
 */

#include <avr/io.h>
#include <util/delay.h>

void mrf_select(void) {
    *mrf_cs_port &= ~(_BV(mrf_cs_pin));
}

void mrf_deselect(void) {
    *mrf_cs_port |= (_BV(mrf_cs_pin));
}

/**
 * use with mrf_reset(&PORTB, PINB5);
 */
void mrf_reset(volatile uint8_t *port, uint8_t pin) {
    *port &= ~(1 << pin);
    _delay_ms(10);  // just my gut
    *port |= (1 << pin);  // active low biatch!
    _delay_ms(20);  // from manual
}


/**
 * Internal spi handling, works on both avr tiny, with USI,
 * and also regular hardware SPI.
 *
 * For regular hardware spi, requires the spi hardware to already be setup!
 * (TODO, you can handle that yourself, or even, have a compile time flag that
 * determines whether to use internal, or provided spi_tx/rx routines)
 */
uint8_t spi_tx(uint8_t cData) {

#if defined(SPDR)
    // AVR platforms with "regular" SPI hardware

    /* Start transmission */
    SPDR = cData;
    /* Wait for transmission complete */
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
#elif defined (USIDR)
    // AVR platforms with USI interfaces, capable of SPI
        /* Start transmission */
    USIDR = cData;
    USISR = (1 << USIOIF);
    do {
        USICR = (1 << USIWM0) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
    } while ((USISR & (1 << USIOIF)) == 0);
    return USIDR;
//#else - stupid netbeans doesn't find the right defines :(
//#error "You don't seem to have any sort of spi hardware!"
#endif
}


