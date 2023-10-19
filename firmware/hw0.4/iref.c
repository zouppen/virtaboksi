// This is a separate C module since we turn off some warnings.

#include <stm8.h>
#include "board.h"

// Disable EVELYN the modified DOG warning. No problem if the GPIO
// reads get cleaned if there's nothing to run in BOARD_IREF_OFF when
// the board has no IREF.
#pragma save
#pragma disable_warning 110

void iref_maybe_off(void)
{
	if (!(STATE(PIN_OUT1) || STATE(PIN_OUT2) || STATE(PIN_OUT3))) {
		BOARD_IREF_OFF;
	}
}

void iref_on(void)
{
	BOARD_IREF_ON;
}

#pragma restore
