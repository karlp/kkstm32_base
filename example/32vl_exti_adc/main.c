/* base headers */
#include "stdint.h"

/* libstm32vl_discovery headers */
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"


#define GPIO_HIGH(a,b) 		a->BSRR = b
#define GPIO_LOW(a,b)		a->BRR = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b 

/* hardware configuration */

#define LED_PORT GPIOC
#define LED_BLUE GPIO_Pin_8  // port doesn't matter...
#define LED_GREEN GPIO_Pin_9

static inline void switch_leds_on(void)
{
    GPIO_LOW(LED_PORT, LED_GREEN);
    GPIO_HIGH(LED_PORT, LED_BLUE);
}

static inline void switch_leds_off(void)
{
    GPIO_HIGH(LED_PORT, LED_GREEN);
    GPIO_LOW(LED_PORT, LED_BLUE);
}

/**
 * We want to use the ADC in this app, which needs the HSI, which runs at 16MHz,
 * by default the 32L starts up in voltage range 2, which only allows 8Mhz 
 * without inserting wait states for flash and so on.
 */
void SystemInit(void)
{
#if 0
    // we need clocks to reach the power part...
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

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
    RCC_PLLCmd(DISABLE);
#endif
    // we don't really need to wait for them to shut down...

    // not sure how to use this yet...
    // SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
    //SCB->VTOR = FLASH_BASE | 0x0; /* Vector Table Relocation in Internal FLASH. */
#endif
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

    /* Enable GPIOs clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure the GPIO_LED pins */
    static GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = LED_GREEN | LED_BLUE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_LOW(LED_PORT, LED_GREEN);
    GPIO_LOW(LED_PORT, LED_BLUE);


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

#if readytouse
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_192Cycles);
    ADC_DelaySelectionConfig(ADC1, ADC_DelayLength_Freeze);

    ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Wait until ADC1 ON status */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
        ;
#endif
}


volatile int button_pressed;

void EXTI0_IRQHandler(void) {
    button_pressed++;
    // clear flag...
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void setup_button_irqs(void)
{
    // AFIO is used for EXTI stuff..
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef button;
    button.GPIO_Mode = GPIO_Mode_IN_FLOATING;
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
    setup_button_irqs();

    uint64_t lasttime = millis();
    while (1) {
        if (millis() - blink_speed_ms > lasttime) {
            if (led_state & 1) {
                switch_leds_on();
            } else {
                switch_leds_off();
            }
            led_state ^= 1;
            lasttime = millis();
        }

        if (button_pressed) {
            button_pressed = 0;
            blink_speed_ms >>= 1;
            if (blink_speed_ms <= 50) {
                blink_speed_ms = 1000;
            }
        }

        // start and wait for adc to convert...
#if readytouse
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
#endif
    }
}
