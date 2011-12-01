/**
 * Karl Palsson, 2011.  Compiled from all sorts of sources, with wisdom
 * from places untold and untracked.  Considered to be released in the public 
 * domain.
 * 
 * Handles setting up data in ram, zeroing bss, and calling SystemInit for clocks
 * then jumping to main
 */

/**
 * Expected to be provided by your application, naturally :)
 * @return 
 */
extern int main(void);

/* provided by the linker script */
extern unsigned long _estack;
extern unsigned long __text_end;
extern unsigned long __data_start;
extern unsigned long __data_end;
extern unsigned long __bss_start;
extern unsigned long __bss_end;


/**
 * Cortex M3 core interrupt handlers
 * 
 * We provide basic implementations of the core of them, but "weak" so that
 * simply declaring them again will count as implementing them
 */
void Reset_Handler(void);
void NMI_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void MemManage_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));
// Define the rest based on Chip type, ST, NXP, etc
// TODO - put in all the rest here, hopefully somehow magically so that the rest 
// of this file can be shared...


void Dummy_Handler(void) {
    while(1) {
        ;
    }
}


/**
 * STM32L1xx starts up running on the MSI, at around 2.097MHz
 * It also is running on "medium" power, 1.5V, with a max clock on 
 * zero wait state flash of only 8Mhz
 * That's enough to run, and handle setting clocks in main().
 * Or, you can provide a SystemInit() to do that sort of thing,
 * which will replace this routine.
 */
void SystemInit(void) __attribute__ ((weak));
void SystemInit(void) {
    ;
}

void __attribute__((noreturn, naked)) Reset_Handler() {
    unsigned long *src;
    unsigned long *dest;
    
    src = &__text_end;
    dest = &__data_start;
    if (src != dest)
        while(dest < &__data_end)
            *(dest++) = *(src++);
 
    dest = &__bss_start;
    while(dest < &__bss_end)
        *(dest++) = 0;
    
	SystemInit();
	main();
}


// TODO - ideally, put these cm3 vectors in one section, 
// and allow other chip type vectors to go in a different section, that the 
// linker would append together....
void *vector_table[] __attribute__ ((section(".vectors"))) = {
	&_estack,
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,
};