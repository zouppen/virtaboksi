#include <stdint.h>
#include "timers.h"

// Used to postpone sleeping for any reason, including waiting for
// serial traffic.
static volatile uint16_t stay_awake = 0;

void timers_stay_awake(uint16_t const a)
{
	uint16_t b = stay_awake;
	stay_awake = a > b ? a : b;
}

bool timers_tick(void) {
	return stay_awake && --stay_awake;
}
