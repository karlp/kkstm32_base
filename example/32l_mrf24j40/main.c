/* base headers */
#include "stdint.h"

#include "misc.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_spi.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_usart.h"

#include "32l_mrf24j40_conf.h"
#include "systick_ms.h"
#include "lib_mrf24j.h"


static inline void switch_leds_on(void)
{
    GPIO_HIGH(GPIOB, LED_GREEN);
    GPIO_HIGH(GPIOB, LED_BLUE);
}

static inline void switch_leds_off(void)
{
    GPIO_LOW(GPIOB, LED_GREEN);
    GPIO_LOW(GPIOB, LED_BLUE);
}

/**
 * by default the 32L starts up in voltage range 2, which only allows 8Mhz 
 * without inserting wait states for flash and so on, but we want to run at 16Mhz,
 * so do all the good setup stuff...
 */
void SystemInit(void)
{
    // we need clocks to reach the power part...
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Select the Voltage Range 1 (1.8 V) */
    PWR->CR = PWR_CR_VOS_0;

    /* Wait Until the Voltage Regulator is ready */
    while ((PWR->CSR & PWR_CSR_VOSF) != RESET)
        ;

    /* HSI is 16mhz RC clock directly fed to SYSCLK (rm00038, figure 9) */

    //#define USE_RAW_CMSIS
#ifdef USE_RAW_CMSIS
    // Do it all by hand with RCC->CR and so on...
#else // use stm32 lib methods...
    /* enable the HSI clock (high speed internal) */
    RCC_HSICmd(ENABLE);

    /* wail til HSI ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
        ;

    /* now switch to this source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    // Turn off unneeded clocks...
    // We're not using LSE yet, but we might want to experiment with it...
    RCC_HSEConfig(RCC_HSE_OFF);
    RCC_LSICmd(DISABLE);
    RCC_MSICmd(DISABLE);
    RCC_PLLCmd(DISABLE);
#endif
    // we don't really need to wait for them to shut down...

    // not sure how to use this yet...
    // SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
    //SCB->VTOR = FLASH_BASE | 0x0; /* Vector Table Relocation in Internal FLASH. */
}

void setup_gpios(void)
{
    /* configure gpios */
    static GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Configure the GPIO_LED pins  LD3 & LD4, plus the MRF reset pin */
    GPIO_InitStructure.GPIO_Pin = LED_GREEN | LED_BLUE | PIN_MRF_RESET;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_LOW(GPIOB, LED_GREEN);
    GPIO_LOW(GPIOB, LED_BLUE);

    // setup my chip select... on PA4
    GPIO_InitStructure.GPIO_Pin = PIN_MRF_CHIPSELECT;
    GPIO_Init(PORT_MRF_CHIPSELECT, &GPIO_InitStructure);
    GPIO_HIGH(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
    
}

/**
 * We'll use the ADC to send some values, maybe....
 */
void setup_adc(void)
{
    // Setup an adc...
    ADC_InitTypeDef adcinit;

    /* Enable ADC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    // all defaults...
    ADC_StructInit(&adcinit);
    ADC_Init(ADC1, &adcinit);

    //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_192Cycles);
    ADC_DelaySelectionConfig(ADC1, ADC_DelayLength_Freeze);

    ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Wait until ADC1 ON status */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
        ;
}

int kkputc(int ch) {
    USART_SendData(USART2, (uint8_t) ch);

    /* Loop until transmit data register is empty */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        ;

    if (ch == '\n') {
        kkputc('\r');
    }
    return ch;
}

int kkwrite(int file, char *ptr, int len)
{
    int todo;

    for (todo = 0; todo < len; todo++) {
        kkputc(*ptr++);
    }
    return len;
}

void phex1(unsigned char c)
{
        kkputc(c + ((c < 10) ? '0' : 'A' - 10));
}

void phex(unsigned char c)
{
        phex1(c >> 4);
        phex1(c & 15);
}

void phex16(unsigned int i)
{
        phex(i >> 8);
        phex(i);
}
     

int kkputs(char *str) {
    char c;
    while ((c = *str++) != '\0')
           kkputc(c);
    return 0;
}

void setup_usart(void) {
    // enable clocks for usart2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    // Setup Alternate Functions to get usart2 out on PA2/PA3...
    GPIO_InitTypeDef usart_af;
    usart_af.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    usart_af.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &usart_af);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    
    
    USART_ClockInitTypeDef usart_clocks;
    USART_ClockStructInit(&usart_clocks);
    usart_clocks.USART_Clock = USART_Clock_Enable;
    USART_ClockInit(USART2, &usart_clocks);
    
    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate = 57600;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart_init);
    USART_Cmd(USART2, ENABLE);
}

// we're using spi2
#define SPIPORT GPIOB
void setup_spi(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  // GPIO_PinAFConfig(SPIPORT, GPIO_PinSource12, GPIO_AF_SPI2);  // nss
  GPIO_PinAFConfig(SPIPORT, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIPORT, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(SPIPORT, GPIO_PinSource15, GPIO_AF_SPI2);

  GPIO_InitTypeDef gpio;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
  gpio.GPIO_Speed = GPIO_Speed_40MHz;

  /* basic config for all pins.. */
  //gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(SPIPORT, &gpio);
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_InitTypeDef spi;
  spi.SPI_Mode = SPI_Mode_Master;
  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi.SPI_DataSize = SPI_DataSize_8b;
  spi.SPI_NSS = SPI_NSS_Soft;
  // Fpclk / 16, should be safe enough, mrf says it goes to 20Mhz
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  // These seem to be correct with MRF manual
  spi.SPI_CPOL = SPI_CPOL_Low;
  spi.SPI_CPHA = SPI_CPHA_1Edge;
  spi.SPI_FirstBit = SPI_FirstBit_MSB;
  
  SPI_Init(SPI2, &spi);
  SPI_Cmd(SPI2, ENABLE);

  /* Enable the Rx buffer not empty interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable the SPI Error interrupt */
//  SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
  /* Data transfer is performed in the SPI interrupt routine */
  /* Enable the SPI peripheral */
}

uint8_t spi_tx(uint8_t cData) {
    // Wait until it's 1, so we can write in
    while ((SPI2->SR & SPI_SR_TXE) == 0)
        ;
    SPI2->DR = cData;
    // wait until it's 1, so we can read out
    while ((SPI2->SR & SPI_SR_RXNE) == 0)
        ;
    return SPI2->DR;
}


/*
void EXTI2_IRQHandler(void) {
    // clear flag...
    EXTI_ClearITPendingBit(EXTI_Line2);
    kkputs("[I]");
    mrf_interrupt_handler();
}
*/

void setup_mrf_irqs(void)
{
    GPIO_InitTypeDef mrf_irq;
    mrf_irq.GPIO_Mode = GPIO_Mode_IN;
    mrf_irq.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOB, &mrf_irq);

    EXTI_InitTypeDef exti;
    exti.EXTI_Line = EXTI_Line2;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = EXTI2_IRQn;
    // These are actually against each other, can't have 4 bits for both of them...
    nvic.NVIC_IRQChannelPreemptionPriority = 0xf; // lowest priority...
    nvic.NVIC_IRQChannelSubPriority = 0xf;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);
}

void handle_rx(mrf_rx_info_t *rxinfo, uint8_t *rx_buffer) {
    kkputs("Received a packet bytes long\n");
    phex(rxinfo->frame_length);
    kkputs("Packet data:\n");
    for (int i = 0; i <= rxinfo->frame_length; i++) {
        phex(rx_buffer[i]);
    }
    kkputs("\nLQI=");
    phex(rxinfo->lqi);
    kkputs(" RSSI=");
    phex(rxinfo->rssi);
}
 
void handle_tx(mrf_tx_info_t *txinfo) {
    if (txinfo->tx_ok) {
        kkputs("TX went ok, got ack\n");
    } else {
        kkputs("TX failed after %d retries\n");
        phex(txinfo->retries);
    }
}

int main(void)
{
    static unsigned int led_state = 0;

    systick_ms_init();
    int blink_speed_ms = 400;

    setup_gpios();
    setup_adc();
    setup_usart();
    kkputs("hello karl...\n");
    //setup_spi();
//    setup_mrf_irqs();

    uint64_t lasttime = millis();
    kkputs("ok karl, about to write the pan...\n");

    mrf_reset();
    mrf_deselect();

    kkputs("ok, time for init...\n");
//    mrf_init();
    // set the pan id to use
//    mrf_pan_write(0xcafe);
    // set our address
//    mrf_address16_write(0x3232);
    kkputs("setup mrf address");
    
    while (1) {
        // Call this pretty often, at least as often as you expect to be receiving packets
//        mrf_check_flags(&handle_rx, &handle_tx);
        if (millis() - blink_speed_ms > lasttime) {
            if (led_state & 1) {
                switch_leds_on();
                kkputc('O');
            } else {
                switch_leds_off();
                kkputc('o');
            }
            led_state ^= 1;
            lasttime = millis();
        }

        // start and wait for adc to convert...
        ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_192Cycles);
        ADC_SoftwareStartConv(ADC1);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0)
            ;

        uint16_t pot_val = ADC_GetConversionValue(ADC1);
        if (pot_val > 0x700) {
//            GPIO_HIGH(GPIOA, GPIO_Pin_4);
        } else {
//            GPIO_LOW(GPIOA, GPIO_Pin_4);
        }
    }
}
