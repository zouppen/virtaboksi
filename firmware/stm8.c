#include <stm8.h>
#include <stdbool.h>

bool stm8_uart1_baudrate(uint16_t rate)
{
	// Divide cpu freq with baud rate, rounding to nearest.
	uint32_t const freq = F_CPU;
	uint32_t const div = (freq + (rate >> 1))/rate;

	if (div > 0xffff) return false;
	if (div < 16) return false;

	// Pick up nibbles and write in correct order (BRR2 first)
	UART1_BRR2 = ((div & 0xf000) >> 8) | (div & 0x000f);
	UART1_BRR1 = (div & 0x0ff0) >> 4;

	return true;
}
