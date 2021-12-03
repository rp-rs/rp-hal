/*
 * Memory set-up for the Raspberry Pi Pico.
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

SECTIONS {
    /* ### Picotool 'Binary Info' Header Block
     *
     * Picotool only searches the second 256 bytes of Flash for this block, but
     * that's where our vector table is. We squeeze in this block after the
     * vector table, but before .text.
     */
    .bi_header : ALIGN(4)
    {
        KEEP(*(.bi_header));
        /* Keep this block a nice round size */
        . = ALIGN(4);
    } > FLASH
} INSERT BEFORE .text;

/* Move _stext, to make room for our new section */
_stext = ADDR(.bi_header) + SIZEOF(.bi_header);

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
     *
     * Picotool looks through this block (as we have pointers to it in our header) to find interesting information.
     */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

