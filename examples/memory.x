/* Memory layout for the RP2040 (Raspberry Pi Pico / Pico W, 2 MB flash).
 *
 * - BOOT2: the 256-byte second-stage bootloader. Its contents are provided by
 *   `embassy-rp` (the `.boot2` section), and placed here by `link-rp.x`.
 * - FLASH: the rest of flash, where your program lives (XIP).
 * - RAM:   264 KB SRAM.
 */
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

