/*
 * Memory set-up for the Pimoroni Pico Explorer.
 *
 * Copyright (c) The RP-RS Developers, 2021
 * Licensed as MIT or Apache 2.0 as per the crate README.md
 */

MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

/* We insert these extra sections into the parent linker script (typically link.x from cortex-m-rt). */
SECTIONS {

    /* ### Boot loader 
     *
     * This section must come first because picotool can't handle out-of-order
     * sections.
     */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2

} INSERT BEFORE .vector_table;
