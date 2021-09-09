MEMORY {
	FLASH	: ORIGIN = 0x8000000,	LENGTH = 256K
	SRAM	: ORIGIN = 0x20000000,	LENGTH = 32K
}

SECTIONS {
    .text : {
        *(.VectorTable)
        *(.text)
    } >FLASH

    .data (NOLOAD) : {
        . = . + 0x400; /* Stack size */
        _StackEnd = .;
        *(.data)
    } >SRAM
}