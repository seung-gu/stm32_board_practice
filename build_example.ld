/******************************************************************************
 *
 * Linker configuration file for STM32F411
 *
 *
 *****************************************************************************/

MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x00080000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000
}

SECTIONS
{
    .text :
    {
        _text = .;
        *(.isr_vector)
        *(.text)
        *(.rodata)
        _etext = .;
    } > FLASH

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _data = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM
}
