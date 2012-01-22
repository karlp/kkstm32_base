/* 
 * File:   32l_mrf24j40_conf.h
 * Author: karlp
 *
 * Created on December 9, 2011, 11:00 PM
 */

#ifndef MRF24J40_CONF_H
#define	MRF24J40_CONF_H

#ifdef	__cplusplus
extern "C" {
#endif

#define GPIO_HIGH(a,b) 		a->BSRRL = b
#define GPIO_LOW(a,b)		a->BSRRH = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b 

/* hardware configuration */
// identical...
//# define LED_BLUE (1 << 6) /* port B, pin 6 */
//# define LED_GREEN (1 << 7) /* port B, pin 7 */
#define LED_BLUE GPIO_Pin_6  // port doesn't matter...
#define LED_GREEN GPIO_Pin_7
#define PORT_MRF_RESET GPIOB  // PB10
#define PIN_MRF_RESET GPIO_Pin_10  // PB10

#define PORT_MRF_CHIPSELECT GPIOA
#define PIN_MRF_CHIPSELECT GPIO_Pin_4  // PA4

// we're using spi2
#define PORT_SPI GPIOB



#ifdef	__cplusplus
}
#endif

#endif	/* MRF24J40_CONF_H */

