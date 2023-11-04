#pragma once

#include <stdint.h>
#include <stdbool.h>

// Prescaler register: f/2^x = 1 MHz, e.g. 16MHz/2^4 = 1 MHz
#if   F_CPU == 16000000
#define TICK_PRESCALE 4
#elif F_CPU ==  8000000
#define TICK_PRESCALE 3
#elif F_CPU ==  4000000
#define TICK_PRESCALE 2
#elif F_CPU ==  2000000
#define TICK_PRESCALE 1
#else
#error No support for this CPU frequency. FIXME by editing timers.h
#endif

// Makes sure the CPU stays out of HALT mode for given number of
// ticks. Never shortens previously-set wake-up
// time. Interrupts must be disabled when calling this.
void timers_stay_awake_IM(uint16_t const a);

// Reports that one tick has elapsed. Returns true if the timer
// still runs.
bool timers_tick_IM(void);
