/* 
 * File:   stm32l1xx_vectors.h
 * Author: karlp
 * considered BSD/MIT license, but the table text was all from the datasheets.
 * Contains a vector table, weakly defined, for _only_ the STM32L1xx specific 
 * interrupt vectors.
 * ie, the ones that are device specifc, _after_ the SYSTICK handler....
 * This is intended to be used to get the entries in the vector table filled in...
 * Just include this file at least once..
 * 
 * Created on December 3, 2011, 11:49 PM
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
void USB_HP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void USB_LP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void DAC_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void COMP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void LCD_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM9_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM10_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM11_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
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
void USB_FS_WKUP_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM6_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));
void TIM7_IRQHandler(void) __attribute__((weak, alias("Dummy_Handler")));

void *stm32l1xx_vector_table[] __attribute__((section(".vectors.stm32l1xx"))) = {
    WWDG_IRQHandler, /* Window Watchdog interrupt */
    PVD_IRQHandler, /* PVD through EXTI Line detection */
    TAMPER_STAMP_IRQHandler, /* Tamper and TimeStamps through the EXTI line */
    RTC_WKUP_IRQHandler, /* RTC Wakeup through the EXTI line */
    FLASH_IRQHandler, /* FLASH global interrupt */
    RCC_IRQHandler, /* RCC global interrupt */
    EXTI0_IRQHandler, /* EXTI Line0 interrupt */
    EXTI1_IRQHandler, /* EXTI Line1 interrupt */
    EXTI2_IRQHandler, /* EXTI Line2 interrupt */
    EXTI3_IRQHandler, /* EXTI Line3 interrupt */
    EXTI4_IRQHandler, /* EXTI Line4 interrupt */
    DMA1_Channel1_IRQHandler, /* DMA1 Channel 1 */
    DMA1_Channel2_IRQHandler, /* DMA1 Channel 2 */
    DMA1_Channel3_IRQHandler, /* DMA1 Channel 3 */
    DMA1_Channel4_IRQHandler, /* DMA1 Channel 4 */
    DMA1_Channel5_IRQHandler, /* DMA1 Channel 5 */
    DMA1_Channel6_IRQHandler, /* DMA1 Channel 6 */
    DMA1_Channel7_IRQHandler, /* DMA1 Channel 6 */
    ADC1_IRQHandler, /* ADC1 global interrupt*/
    USB_HP_IRQHandler, /* USB High priority interrupt */
    USB_LP_IRQHandler, /* USB Low priority interrupt */
    DAC_IRQHandler, /* DAC interrupt */
    COMP_IRQHandler, /* Comparator wakeup through EXTI line (21 and 22) interrupt */
    EXTI9_5_IRQHandler, /* EXTI Line[9:5] interrupts */
    LCD_IRQHandler, /* LCD global interrupt */
    TIM9_IRQHandler, /* TIM9 global interrupt */
    TIM10_IRQHandler, /* TIM10 global interrupt */
    TIM11_IRQHandler, /* TIM11 global interrupt priority */
    TIM2_IRQHandler, /* TIM2 global interrupt */
    TIM3_IRQHandler, /* TIM3 global interrupt */
    TIM4_IRQHandler, /* TIM4 global interrupt */
    I2C1_EV_IRQHandler, /* I2C1 event interrupt */
    I2C1_ER_IRQHandler, /* I2C1 error interrupt */
    I2C2_EV_IRQHandler, /* I2C2 event interrupt */
    I2C2_ER_IRQHandler, /* I2C2 error interrupt*/
    SPI1_IRQHandler, /* SPI1 global interrupt */
    SPI2_IRQHandler, /* SPI2 global interrupt */
    USART1_IRQHandler, /* USART1 global interrupt */
    USART2_IRQHandler, /* USART2 global interrupt */
    USART3_IRQHandler, /* USART3 global interrupt */
    EXTI15_10_IRQHandler, /* EXTI Line[15:10] interrupts */
    RTC_Alarm_IRQHandler, /* RTC Alarms (A and B) through EXTI line interrupt */
    USB_FS_WKUP_IRQHandler, /* USB Device FS Wakeup through EXTI line interrupt */
    TIM6_IRQHandler, /* TIM6 global interrupt */
    TIM7_IRQHandler /* TIM7 global interrupt */
};
