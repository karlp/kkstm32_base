/*
 * Karl Palsson, 2011
 * This is a memory map for the STM32L15xxB parts
 * It is considered to be released into the public domain
 */
ENTRY(Reset_Handler)

MEMORY
{
    RAM (rxw)     : ORIGIN = 0x20000000, LENGTH = 16K
}

INCLUDE generic_sections_sram.ld
