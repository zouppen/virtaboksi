#pragma once

#include <stdint.h>
#include <stm8.h>

#define settings (*(settings_t *)(EEPROM_START_ADDR))

typedef struct {
	char initialized;
	uint32_t baud_rate;
} settings_t;

// Enable EEPROM writes and initialize settings to default values if
// initialized = false.
void settings_init(void);
