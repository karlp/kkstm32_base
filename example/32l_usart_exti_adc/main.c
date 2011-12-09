/* base headers */
#include "stdint.h"

/* libstm32l_discovery headers */
#include "misc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_usart.h"


#define GPIO_HIGH(a,b) 		a->BSRRL = b
#define GPIO_LOW(a,b)		a->BSRRH = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b 

/* hardware configuration */

#define GPIOB_MODER (GPIOB + 0x00) /* port mode register */
#define GPIOB_ODR (GPIOB + 0x14) /* port output data register */

// identical...
//# define LED_BLUE (1 << 6) /* port B, pin 6 */
//# define LED_GREEN (1 << 7) /* port B, pin 7 */
#define LED_BLUE GPIO_Pin_6  // port doesn't matter...
#define LED_GREEN GPIO_Pin_7

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
 * We want to use the ADC in this app, which needs the HSI, which runs at 16MHz,
 * by default the 32L starts up in voltage range 2, which only allows 8Mhz 
 * without inserting wait states for flash and so on.
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

volatile uint64_t ksystick;

uint64_t millis(void)
{
    return ksystick;
}

void SysTick_Handler(void)
{
    ksystick++;
}

void setup_gpios(void)
{
    /* configure gpios */
    static GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    /* Configure the GPIO_LED pins  LD3 & LD4*/
    GPIO_InitStructure.GPIO_Pin = LED_GREEN | LED_BLUE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_LOW(GPIOB, LED_GREEN);
    GPIO_LOW(GPIOB, LED_BLUE);


    // setup my own green led... on PC3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // setup my own red led... on PA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void setup_adc(void)
{
    // Setup an adc...
    ADC_InitTypeDef adcinit;

    /* Enable ADC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    //ADC_DeInit(ADC1);
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

volatile int button_pressed;

void EXTI0_IRQHandler(void) {
    button_pressed++;
    // clear flag...
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void setup_button_irqs(void)
{
    GPIO_InitTypeDef button;
    button.GPIO_Mode = GPIO_Mode_IN;
    button.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &button);

    EXTI_InitTypeDef exti;
    exti.EXTI_Line = EXTI_Line0;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    // These are actually against each other, can't have 4 bits for both of them...
    nvic.NVIC_IRQChannelPreemptionPriority = 0xf; // lowest priority...
    nvic.NVIC_IRQChannelSubPriority = 0xf;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);
}

int main(void)
{
    static unsigned int led_state = 0;

    RCC_ClocksTypeDef clockinfo;
    RCC_GetClocksFreq(&clockinfo);
    // regardless of clock speed this gives us 1000 ticks per second
    SysTick_Config(clockinfo.SYSCLK_Frequency / 1000);
    int blink_speed_ms = 400;

    setup_gpios();
    setup_adc();
    setup_usart();
    setup_button_irqs();

    kkputs("hello karl...\n");
    uint64_t lasttime = millis();
    while (1) {
        if (millis() - blink_speed_ms > lasttime) {
            if (led_state & 1) {
                switch_leds_on();
                kkputs("O");
            } else {
                switch_leds_off();
                kkputs("o");
            }
            led_state ^= 1;
            GPIO_TOGGLE(GPIOC, GPIO_Pin_3);
            lasttime = millis();
        }

        if (button_pressed) {
            button_pressed = 0;
            kkputs("button was pressed!\n");
            blink_speed_ms >>= 1;
            if (blink_speed_ms <= 50) {
                blink_speed_ms = 1000;
            }
        }

        // start and wait for adc to convert...
        ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_192Cycles);
        ADC_SoftwareStartConv(ADC1);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0)
            ;

        uint16_t pot_val = ADC_GetConversionValue(ADC1);
        if (pot_val > 0x700) {
            GPIO_HIGH(GPIOA, GPIO_Pin_4);
        } else {
            GPIO_LOW(GPIOA, GPIO_Pin_4);
        }
    }
}
