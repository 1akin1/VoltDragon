/*
 * Linker script for the STM32F407VG (1 MiB flash, 128 KiB SRAM1+SRAM2).
 *
 * RAM layout (low to high):
 *   .noinit  - survives warm resets; placed first so its address does not
 *              move when .data/.bss grow between firmware versions
 *   .data    - initialised data, copied from flash by Reset_Handler
 *   .bss     - zero-initialised data, cleared by Reset_Handler
 *   free     - unused RAM (no heap: dynamic allocation is not permitted)
 *   stack    - main stack, grows down from the end of RAM
 */

ENTRY(Reset_Handler)

/* Minimum main-stack size; the link fails if it does not fit. */
_min_stack_size = 0x1000;

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

/*
 * Only flash contents form loadable segments. RAM sections are initialised by
 * Reset_Handler, so loaders (GDB, Renode) must not touch RAM: zero-filling it
 * would wipe the .noinit reset record.
 */
PHDRS
{
    flash   PT_LOAD;
    data    PT_LOAD;
}

SECTIONS
{
    /* Vector table must be first in flash: the core fetches SP and PC from it. */
    .isr_vector :
    {
        . = ALIGN(512);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH :flash

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* C++ style constructors are not supported by our startup code. */
    .init_array : { *(.preinit_array*) *(.init_array*) } > FLASH :flash
    ASSERT(SIZEOF(.init_array) == 0, "static constructors are not supported")

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        _snoinit = .;
        *(.noinit)
        *(.noinit*)
        . = ALIGN(4);
        _enoinit = .;
    } > RAM :NONE

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH :data

    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM :NONE

    /* Symbols expected by newlib's nosys stubs; there is no heap. */
    end = _ebss;
    _end = _ebss;

    ASSERT(_ebss + _min_stack_size <= _estack, "not enough RAM left for the main stack")

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
