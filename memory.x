MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    /*
     * RAM consists of 4 banks, SRAM0-SRAM3, with a striped mapping.
     * This is usually good for performance, as it distributes load on
     * those banks evenly.
     */
    RAM : ORIGIN = 0x20000000, LENGTH = 256K
    /*
     * RAM banks 4 and 5 use a direct mapping. They can be used to have
     * memory areas dedicated for some specific job, improving predictability
     * of access times.
     * Example: Separate stacks for core0 and core1.
     */
    SRAM4 : ORIGIN = 0x20040000, LENGTH = 4k
    SRAM5 : ORIGIN = 0x20041000, LENGTH = 4k

    /* SRAM banks 0-3 can also be accessed directly. However, those ranges
       alias with the RAM mapping, above. So don't use them at the same time!
    SRAM0 : ORIGIN = 0x21000000, LENGTH = 64k
    SRAM1 : ORIGIN = 0x21010000, LENGTH = 64k
    SRAM2 : ORIGIN = 0x21020000, LENGTH = 64k
    SRAM3 : ORIGIN = 0x21030000, LENGTH = 64k
    */
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

/* Per-core (thread) data into flash */
SECTIONS {
    .tdata : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(__tdata_start = .);
        *(.tdata .tdata.*);
        . = ALIGN(4);
        PROVIDE(__tdata_end = .);
    } > FLASH
    PROVIDE(__tdata_len = __tdata_end - __tdata_start);
} INSERT AFTER .data;

/* Size per-core state and allocate bss space for each core */
SECTIONS {
    .tbss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(__tbss_start = .);
        *(.tbss .tbss.*);
        *(.tcommon);
        . = ALIGN(4);
        PROVIDE(__tbss_end = .);
    } > RAM
    PROVIDE(__tbss_len = __tbss_end - __tbss_start);

    .tls_state (NOLOAD) : ALIGN(4) {
        PROVIDE(TLS_CORE_0 = ALIGN(4));
        . += __tdata_len + __tbss_len;
        PROVIDE(TLS_CORE_1 = ALIGN(4));
        . += __tdata_len + __tbss_len;
    } > RAM
} INSERT AFTER .bss;
