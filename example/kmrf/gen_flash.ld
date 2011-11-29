
ENTRY(Reset_Handler)

/* 
this is for stm32L medium 
However, because only the _lengths_ change, we can use the sections without change...
*/
MEMORY
{
    FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rxw)     : ORIGIN = 0x20000000, LENGTH = 16K
}

/* _estack = ADDR(RAM) + SIZEOF(RAM); */
PROVIDE(_estack = 0x20004000);

SECTIONS
{
    . = ORIGIN(FLASH);
    .text : 
    {
        *(.vectors)    /* Vector table */
        *(.text)      /* Program code */
        *(.rodata)    /* Read only data */
        *(.rodata*)
        __text_end = .;
    } >FLASH

    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data :
    {
        /* This is used by the startup in order to initialize the .data secion */
        PROVIDE (__data_start = .);
        *(.data)
        *(.data.*)
        /* This is used by the startup in order to initialize the .data secion */
        PROVIDE (__data_end = .);
    } >RAM AT >FLASH


    .bss :
    {
        PROVIDE(__bss_start = .);
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(__bss_end = .);
    } >RAM

    . = ALIGN(4);

    _stack_start = .;

}

_end = .;
