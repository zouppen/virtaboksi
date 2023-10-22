#include <stm8.h>
#include "board.h"

bool serial_is_transmitting(void) {
	return STATE(PIN_TX_EN);
}
