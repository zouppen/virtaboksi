#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DEBOUNCE_MS 200
#define STARTUP_DEBOUNCE_MS 500
#define PANIC_OFF_MS 1000
#define MINIMUM_WAKEUP_MS 100
#define SERIAL_KEEPALIVE_MS 1000

// Prescaler register: f/2^x = 125 kHz, e.g. 2MHz/2^4 = 125 kHz
#if   F_CPU == 16000000
#define TICK_PRESCALE 7
#elif F_CPU ==  8000000
#define TICK_PRESCALE 6
#elif F_CPU ==  4000000
#define TICK_PRESCALE 5
#elif F_CPU ==  2000000
#define TICK_PRESCALE 4
#else
#error No support for this CPU frequency. FIXME by editing timers.h
#endif

// Makes sure the CPU stays out of HALT mode for given number of
// milliseconds. Never shortens previously-set wake-up
// time. Interrupts must be disabled when calling this.
void timers_stay_awake_IM(uint16_t const a);

// Reports that one tick (1ms) has elapsed. Returns true if the timer
// still runs.
bool timers_tick_IM(void);
