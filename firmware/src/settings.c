#include "settings.h"

void settings_init(void)
{
	stm8_eeprom_unlock();

	if (settings.initialized != 'Z') {
		settings.baud_rate = BAUD;
		settings.initialized = 'Z';
	}
}
