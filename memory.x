/* Linker script for the STMF051C8T6 */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 8K
}

/* NOTE: Do *NOT* modify `_stack_start` unless you know what you are doing. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
