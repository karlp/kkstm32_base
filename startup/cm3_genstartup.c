/**
 * Karl Palsson, 2011.  Compiled from all sorts of sources, with wisdom
 * from places untold and untracked.  Considered to be released into the public
 * domain.
 *
 * This was helpful:
 * http://dev.frozeneskimo.com/notes/getting_started_with_cortex_m3_cmsis_and_gnu_tools
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

/**
 * The chip specific (STM32L1xx or F1, or LPC, or TI etc) vectors are in a 
 * chip specific file.  by placing them in a subsection, they can be linked in
 * immediately after the core cortex handler table.
 */

void Dummy_Handler(void) {
    while(1) {
        ;
    }
}


/**
 * All the parts start on some sort of an internal clock.
 * STM32F101, 103, 105, 107 start on HSI (~8MHz)
 * STM32F100xx (Value Line) start on HSI (~8MHz)
 * STM32F2xx starts on HSI (~16MHz)
 * STM32F4xx starts on HSI (~16MHz)
 * STM32L1xx starts on MSI (~2MHz)
 * Additionally, the STM32L1xx starts in voltage range 2 (1.5V), and needs
 * to be switched to voltage range 1 before enabling its HSI (~16MHz)
 *
 * Regardless, the default startup clock is enough to run.  However, you may
 * wish to override this function, (simply implement your own) and configure
 * clocks, power regulators, or any other "premain" code you wish to run.
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
#if defined(CONFIG_BOOT_SRAM) && (CONFIG_BOOT_SRAM > 0)
  static const unsigned int fstack = (unsigned int)&_estack;
  __asm__ __volatile__
    (
     "ldr sp, %0\n\t"
     : 
     : "m"(fstack)
     : "sp"
    );
    
  // Not really sure what this is, but the docs make it seem important...
//        SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#define VTOR   0xE000ED08 
#define SRAM_BASE (1<<29)
    *(volatile unsigned int*)VTOR = SRAM_BASE;
#endif
	SystemInit();
	main();
}

/**
 * This table contains the core Coretex vectors, and should be linked first.
 * You should link any chip specific tables after this.
 */
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