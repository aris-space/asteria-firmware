/* STM32H723ZG. Declared here (rather than via embassy's `memory-x`) so SRAM4 in
   the D3 domain is a first-class region — it is the only RAM BDMA can reach, and
   the I2C4 bounce buffer lives there (see `.sram4` in build.rs / bounce_i2c.rs). */
MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 1024K /* BANK_1                  */
    RAM    : ORIGIN = 0x24000000, LENGTH =  320K /* AXI SRAM (D1)           */
    RAM_D3 : ORIGIN = 0x38000000, LENGTH =   16K /* SRAM4 (D3), BDMA-visible */
}
