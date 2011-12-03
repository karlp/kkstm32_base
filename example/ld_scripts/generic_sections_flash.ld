/**
 * Karl Palsson, 2011
 * Considered to be released into the public domain
 *
 * Generic "boot from flash" linker script.
 *
 * This file simply defines how sections should get written.
 * You should INCLUDE this file from another file, that specifies the
 * memory map for the part you are using.
 */

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

    /*
     * This is the initialized data section
     * The program executes knowing that the data is in the RAM
     * but the loader puts the initial values in the FLASH (inidata).
     * One task of "startup" is to copy the initial values from FLASH to RAM.
     */
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

PROVIDE(_estack = ORIGIN(RAM) + LENGTH(RAM) - 4);
