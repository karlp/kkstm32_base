/* base headers */
#include "stdint.h"

/* libstm32l_discovery headers */
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_dac.h"
#include "stm32l1xx_lcd.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_flash.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_dbgmcu.h"

#define GPIO_HIGH(a,b) 		a->BSRRL = b
#define GPIO_LOW(a,b)		a->BSRRH = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b 

/* hardware configuration */

# define GPIOB_MODER (GPIOB + 0x00) /* port mode register */
# define GPIOB_ODR (GPIOB + 0x14) /* port output data register */

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

#define delay()						\
do {							\
  volatile unsigned int i;				\
  for (i = 0; i < 1000000; ++i)				\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)


static void RCC_Configuration(void)
{
  /* HSI is 16mhz RC clock directly fed to SYSCLK (rm00038, figure 9) */

  /* enable the HSI clock (high speed internal) */
  RCC_HSICmd(ENABLE);
  
  /* wail til HSI ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  /* at startup, SYSCLK driven by MSI. set to HSI */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

  // Turn off unneeded clocks...
  // We're not using LSE yet, but we might want to experiment with it...
  RCC_HSEConfig(RCC_HSE_OFF);  
  RCC_LSICmd(DISABLE);
  RCC_MSICmd(DISABLE);
  RCC_PLLCmd(DISABLE);
  // we don't really need to wait for them to shut down...
/*
  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
  {
    while (1) ;
  }
*/
}

volatile uint64_t ksystick;

uint64_t millis(void) {
    return ksystick;
}

void SysTick_Handler(void) {
    ksystick++;
}


volatile uint64_t loopcount = 0;

void main(void)
{
  static RCC_ClocksTypeDef RCC_Clocks;
  static GPIO_InitTypeDef GPIO_InitStructure;
  static unsigned int led_state = 0;

  /* Configure Clocks for Application need */
  // Pretty sure this is taken care of in system_stm32l1xx.c....
  RCC_Configuration();
  
  RCC_ClocksTypeDef clockinfo;
  RCC_GetClocksFreq(&clockinfo);
  // Should be 16Mhz, we want 1ms, and there's 1000 of them in a second.
  SysTick_Config(clockinfo.SYSCLK_Frequency / 1000);
  
  /* configure gpios */

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
  // setup my own reed led... on PA4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
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
  {
  }

    uint64_t lasttime = millis();
    while (1) {
        loopcount++;
        if (millis() - 2000 > lasttime) {
            if (led_state & 1) {
                switch_leds_on();
            } else {
                switch_leds_off();
            }
            led_state ^= 1;
            GPIO_TOGGLE(GPIOC, GPIO_Pin_3);
            lasttime = millis();
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
