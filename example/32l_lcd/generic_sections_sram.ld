SECTIONS
{
	.text(ORIGIN(RAM)):
	{
        *(.vectors)    /* Vector table */
        *(.text)      /* Program code */
        *(.rodata)    /* Read only data */
        *(.rodata*)
        __text_end = .;
	} > RAM

	.rodata :
	{
		. = ALIGN(4);
		_frodata = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		*(.rodata1)
		_erodata = .;
	} > RAM

	.data :
	{
        /* This is used by the startup in order to initialize the .data secion */
        PROVIDE (__data_start = .);
        *(.data)
        *(.data.*)
        /* This is used by the startup in order to initialize the .data secion */
        PROVIDE (__data_end = .);
	} > RAM

	.bss :
	{
        PROVIDE(__bss_start = .);
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(__bss_end = .);
	} > RAM
}

_end = .;

PROVIDE(_estack = ORIGIN(RAM) + LENGTH(RAM) - 4);
