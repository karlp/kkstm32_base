/* 
 * File:   stm32f100x.h
 * Author: karlp
 * considered BSD/MIT license, but the table text was all from the datasheets.
 * Contains a vector table, weakly defined, for _only_ the STM32F100x specific 
 * interrupt vectors.
 * (VALUE LINE! no guarantee that this is correct for the other stm32F1s!)
 * ie, the ones that are device specifc, _after_ the SYSTICK handler....
 * This is intended to be used to get the entries in the vector table filled in...
 * Just include this file at least once..
 * 
 * Created on January 4, 2012, from RM0041, rev 4.
 */

/**
 * This should be defined in your other startup files...
 */
// Something's not quite right.. I was sure this was working before,
// and properly using the Dummy_Handler() defined in cm3_genstartup.c
//extern void Dummy_Handler(void);
void Dummy_Handler(void) __attribute__ ((weak));
void Dummy_Handler(void) {
    while(1)
        ;
}

// Define a weak handler for each of these....
void WWDG_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void PVD_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TAMPER_STAMP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void RTC_WKUP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel4_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel6_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void ADC1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM1_BRK_TIM15_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void USART3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void CEC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM12_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM13_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM14_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void FSMC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void SPI3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void UART4_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void UART5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM6_DAC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM7_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA2_Channel1_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA2_Channel2_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA2_Channel3_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA2_Channel4_5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DMA2_Channel5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));

void *stm32f100x[] __attribute__((section(".vectors.stm32f100x"))) = {
	WWDG_IRQHandler, /* Window Watchdog interrupt */
	PVD_IRQHandler, /* PVD through EXTI Line detection interrupt */
	TAMPER_STAMP_IRQHandler, /* Tamper and TimeStamp through EXTI line interrupts */
	RTC_WKUP_IRQHandler, /* RTC Wakeup through EXTI line interrupt */
	FLASH_IRQHandler, /* Flash global interrupt */
	RCC_IRQHandler, /* RCC global interrupt */
	EXTI0_IRQHandler, /* EXTI Line0 interrupt */
	EXTI1_IRQHandler, /* EXTI Line1 interrupt */
	EXTI2_IRQHandler, /* EXTI Line2 interrupt */
	EXTI3_IRQHandler, /* EXTI Line3 interrupt */
	EXTI4_IRQHandler, /* EXTI Line4 interrupt */
	DMA1_Channel1_IRQHandler, /* DMA1 Channel1 global interrupt */
	DMA1_Channel2_IRQHandler, /* DMA1 Channel2 global interrupt */
	DMA1_Channel3_IRQHandler, /* DMA1 Channel3 global interrupt */
	DMA1_Channel4_IRQHandler, /* DMA1 Channel4 global interrupt */
	DMA1_Channel5_IRQHandler, /* DMA1 Channel5 global interrupt */
	DMA1_Channel6_IRQHandler, /* DMA1 Channel6 global interrupt */
	DMA1_Channel7_IRQHandler, /* DMA1 Channel7 global interrupt */
	ADC1_IRQHandler, /* ADC1 global interrupt */
	0,
	0,
	0,
	0,
	EXTI9_5_IRQHandler, /* EXTI Line[9:5] interrupts */
	TIM1_BRK_TIM15_IRQHandler, /* TIM1 Break and TIM15 global interrupt */
	TIM1_UP_TIM16_IRQHandler, /* TIM1 Update and TIM16 global interrupts */
	TIM1_TRG_COM_TIM17_IRQHandler, /* TIM1 Trigger and Commutation and TIM17 global interrupts */
	TIM1_CC_IRQHandler, /* TIM1 Capture Compare interrupt */
	TIM2_IRQHandler, /* TIM2 global interrupt */
	TIM3_IRQHandler, /* TIM3 global interrupt */
	TIM4_IRQHandler, /* TIM4 global interrupt */
	I2C1_EV_IRQHandler, /* I2C1 event interrupt */
	I2C1_ER_IRQHandler, /* I2C1 error interrupt */
	I2C2_EV_IRQHandler, /* I event interrupt */
	I2C2_ER_IRQHandler, /* I2C2 error interrupt */
	SPI1_IRQHandler, /* SPI1 global interrupt */
	SPI2_IRQHandler, /* SPI2 global interrupt */
	USART1_IRQHandler, /* USART1 global interrupt */
	USART2_IRQHandler, /* USART2 global interrupt */
	USART3_IRQHandler, /* USART3 global interrupt */
	EXTI15_10_IRQHandler, /* EXTI Line[15:10] interrupts */
	RTC_Alarm_IRQHandler, /* RTC Alarms (A and B) through EXTI line interrupt */
	CEC_IRQHandler, /* CEC global interrupt */
	TIM12_IRQHandler, /* TIM12 global interrupt */
	TIM13_IRQHandler, /* TIM13 global interrupt */
	TIM14_IRQHandler, /* TIM14 global interrupt */
	0,
	0,
	FSMC_IRQHandler, /* FSMC global interrupt */
	0,
	TIM5_IRQHandler, /* TIM5 global interrupt */
	SPI3_IRQHandler, /* SPI3 global interrupt */
	UART4_IRQHandler, /* UART4 global interrupt */
	UART5_IRQHandler, /* UART5 global interrupt */
	TIM6_DAC_IRQHandler, /* TIM6 global and DAC underrun interrupts */
	TIM7_IRQHandler, /* TIM7 global interrupt */
	DMA2_Channel1_IRQHandler, /* DMA2 Channel1 global interrupt */
	DMA2_Channel2_IRQHandler, /* DMA2 Channel2 global interrupt */
	DMA2_Channel3_IRQHandler, /* DMA2 Channel3 global interrupt */
	DMA2_Channel4_5_IRQHandler, /* DMA2 Channel4 and DMA2 Channel5 global interrupts */
	DMA2_Channel5_IRQHandler, /* DMA2 Channel5 global interrupt */
};
