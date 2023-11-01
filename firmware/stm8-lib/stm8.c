#include <stdbool.h>
#include "stm8.h"

bool stm8_uart1_baudrate(uint32_t rate)
{
	// Divide cpu freq with baud rate, rounding to nearest.
	uint32_t const freq = F_CPU;
	uint32_t const div = (freq + rate/2)/rate;

	if (div > 0xffff) return false;
	if (div < 16) return false;

	// Pick up nibbles and write in correct order (BRR2 first)
	UART1_BRR2 = ((div >> 8) & 0xf0) | (div & 0x0f);
	UART1_BRR1 = div >> 4;

	return true;
}

void stm8_configure_clock(void)
{
#if   F_CPU == 16000000
	CLK_CKDIVR = 0b00 << 3;
#elif F_CPU ==  8000000
	CLK_CKDIVR = 0b01 << 3;
#elif F_CPU ==  4000000
	CLK_CKDIVR = 0b10 << 3;
#elif F_CPU ==  2000000
	// Default is 0b11 << 3;
#else
#error No support for this CPU frequency. FIXME by editing stm8.c
#endif
}

void stm8_eeprom_unlock(void)
{
    FLASH_DUKR = FLASH_DUKR_KEY1;
    FLASH_DUKR = FLASH_DUKR_KEY2;
    while (!(FLASH_IAPSR & FLASH_IAPSR_DUL));
}

void stm8_eeprom_lock(void)
{
    FLASH_IAPSR &= ~FLASH_IAPSR_DUL;
}
