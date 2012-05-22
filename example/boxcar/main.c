/*
 * Karl Palsson, 2012
 * boxcar project code.
 * 2x analog probes, or digital outs
 * 1x 802.15.4 module (MRF24J40xx)
 * 1x DHT03 temp/humi sensor
 * 1x debug led
 * battery powered, so designed for lower power.  (but not using the actual
 * low power stm32L)
 * board files at: https://github.com/karlp/karlnet/tree/master/nodes/boxcar
 */

#include <stdint.h>
#include <string.h>

/* libstm32vl_discovery headers */
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"


#define GPIO_HIGH(a,b) 		a->BSRR = b
#define GPIO_LOW(a,b)		a->BRR = b
#define GPIO_TOGGLE(a,b) 	a->ODR ^= b 

/* hardware configuration */

#define LED_PORT GPIOA
#define LED_DEBUG GPIO_Pin_2
#define ADC_PORT GPIOA

struct phono_adc_connector {
    uint16_t last_reading;
    uint8_t adc_channel;
    uint16_t adc_detect_pin;
    uint16_t adc_input_pin;
};

struct bstate {
    bool led_on;
    int blink_speed_ms;
    uint64_t last_blink_time;
    struct phono_adc_connector analog_channel1;
    struct phono_adc_connector analog_channel2;
};

static inline void switch_leds_on(void)
{
    GPIO_HIGH(LED_PORT, LED_DEBUG);
}

static inline void switch_leds_off(void)
{
    GPIO_LOW(LED_PORT, LED_DEBUG);
}

/**
 * Check what requirements F100 has for adc usage, and double check default 
 * startup clocks... (Compare with 32L code)
 */
void SystemInit(void)
{
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

void setup_gpios(struct bstate *st) {
    /* Enable GPIOs clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure the GPIO_LED pins */
    static GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = LED_DEBUG;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_PORT, &gpio);
    GPIO_LOW(LED_PORT, LED_DEBUG);
    
    // setup detection pins for phono jacks
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    gpio.GPIO_Pin = st->analog_channel1.adc_detect_pin | st->analog_channel2.adc_detect_pin;
    GPIO_Init(ADC_PORT, &gpio);
}

/**
 * sets up gpios to be adc in, and the adc itself
 */
void setup_adc(struct bstate *st)
{
    static GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = st->analog_channel1.adc_input_pin | st->analog_channel2.adc_input_pin;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_PORT, &gpio);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig (RCC_PCLK2_Div6);  // FIXME
    //ADC_DeInit(ADC1);
    // all defaults...
    ADC_InitTypeDef adcinit;
    ADC_StructInit(&adcinit);
    ADC_Init(ADC1, &adcinit);
    ADC_Cmd(ADC1, ENABLE);
}

/**
 * Task to update the blinking heartbeat led in debug setups
 * @param st
 */
void update_leds(struct bstate *st) {
    if (millis() - st->blink_speed_ms > st->last_blink_time) {
        if (st->led_on & 1) {
            switch_leds_on();
        } else {
            switch_leds_off();
        }
        st->led_on ^= 1;
        st->last_blink_time = millis();
    }
}

/**
 * Basic blocking read.
 * Only reads if the io pin indicates a plug is connected
 * @param adc
 */
void task_get_channel(struct phono_adc_connector *adc) {
    if (GPIO_ReadInputDataBit(ADC_PORT, adc->adc_detect_pin)) {
        // start and wait for adc to convert...
        ADC_RegularChannelConfig(ADC1, adc->adc_channel, 1, ADC_SampleTime_239Cycles5);
        ADC_Cmd(ADC1, ENABLE);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
            ;
        adc->last_reading = ADC_GetConversionValue(ADC1);
    }
}


int main(void)
{
    RCC_ClocksTypeDef clockinfo;
    RCC_GetClocksFreq(&clockinfo);
    // regardless of clock speed this gives us 1000 ticks per second
    SysTick_Config(clockinfo.SYSCLK_Frequency / 1000);
    struct bstate state;
    // FIXME - karl, you need to sort this shit out once and for all...
    //memset(&state, 0, sizeof(state));
    // These are from my schematic....
    state.analog_channel1.adc_channel = ADC_Channel_5;
    state.analog_channel1.adc_detect_pin = GPIO_Pin_6;
    state.analog_channel1.adc_input_pin = GPIO_Pin_5;
    state.analog_channel2.adc_channel = ADC_Channel_3;
    state.analog_channel2.adc_detect_pin = GPIO_Pin_4;
    state.analog_channel2.adc_input_pin = GPIO_Pin_3;
    
    state.blink_speed_ms = 400;
    state.last_blink_time = millis();

    setup_gpios(&state);
    setup_adc(&state);

    while (1) {
        update_leds(&state);
        task_get_channel(&state.analog_channel1);
        task_get_channel(&state.analog_channel2);
    }
}
