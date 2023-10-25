#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS 200
#define STARTUP_DEBOUNCE_MS 500
#define PANIC_OFF_MS 1000
#define MINIMUM_WAKEUP_MS 100
#define SERIAL_KEEPALIVE_MS 1000

// Makes sure the CPU stays out of HALT mode for given number of
// milliseconds. Never shortens previously-set wake-up
// time. Interrupts must be disabled when calling this.
void timers_stay_awake(uint16_t const a);

// Reports that one tick (1ms) has elapsed. Returns true if the timer
// still runs.
bool timers_tick(void);
