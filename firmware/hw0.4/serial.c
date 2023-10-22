#include <stm8.h>
#include "board.h"

void serial_init(void)
{
	// UART configuration
	UART1_CR2 =
		UART_CR2_TEN |   // Enable TX
		UART_CR2_REN |   // Receiver enable
		UART_CR2_RIEN;   // Receiver interrupt enabled
	// UART1_CR3 default is 1 stop bit
	stm8_uart1_baudrate(9600);

	// Turn on rs485 rx and put to receive mode
	OUTPUT(PIN_TX_EN);
	LOW(PIN_TX_EN);
}

bool serial_is_transmitting(void) {
	return STATE(PIN_TX_EN);
}
