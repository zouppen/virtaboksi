/* Firmware for Virtaboksi v0.4 */
#include <stdint.h>
#include <stdbool.h>
#include "stm8.h"

// Pin configuration
#define LED_DEBUG    PD,1
#define SW_BAT       PC,5
#define SW_AWAY      PC,7
#define SW_HOME      PC,6
#define CTRL_PRI     PD,4
#define CTRL_HOME1   PA,3
#define CTRL_HOME2   PA,2
#define LED_AWAY     PC,3
#define LED_HOME     PB,4
#define LED_THIRD    PB,5

// On bootup, have a small pause after bootup before switching loads,
// to avoid oscillation in case of a boot loop.
static volatile uint16_t ctrl_debounce = 500;

void update_outputs(void);

void update_outputs() {
	bool const bat_good = READ(SW_BAT);
	bool const sw_away = !READ(SW_AWAY);
	bool const sw_home = !READ(SW_HOME);
	bool const sw_main = sw_home || sw_away;

	if (!bat_good || !sw_main) {
		// BAT bad or OFF state: all outputs off
		LOW(CTRL_PRI);
		LOW(CTRL_HOME1);
		LOW(CTRL_HOME2);
		// Indicator LEDs
		HIGH(LED_AWAY);
		HIGH(LED_HOME);
		LOW(LED_THIRD);
	} else if (sw_home) {
		// Home: All outputs on
		HIGH(CTRL_PRI);
		HIGH(CTRL_HOME1);
		HIGH(CTRL_HOME2);
		// Indicator LEDs
		HIGH(LED_AWAY);
		LOW(LED_HOME);
		HIGH(LED_THIRD);
	} else {
		// Away: PRI on
		HIGH(CTRL_PRI);
		LOW(CTRL_HOME1);
		LOW(CTRL_HOME2);
		// Indicator LEDs
		LOW(LED_AWAY);
		HIGH(LED_HOME);
		HIGH(LED_THIRD);
	}
}

void debounce_start(void) __interrupt(EXTI3_IRQ) {
	ctrl_debounce = 200;
}

void debounce_check(void) __interrupt(TIM2_OVR_UIF_IRQ) {
	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;
	
	// Blink the LED when in operation
	TOGGLE(LED_DEBUG);

	if (ctrl_debounce) {
		if (!--ctrl_debounce) {
			// Indicator LED turns off
			LOW(LED_DEBUG);

			// Update outputs and then halt the whole CPU
			update_outputs();

			// When done counting, halt the whole CPU
			halt();
		}
	}
}

int main(void)
{
	// Not changing clock speed, running on default of 2 MHz

	// As suggested in chapter 11.5 of RM0016, unused pins are set
	// to pull-up state. We make them all pull-up first, it's safe.
	PA_CR1 = 0xFF;
	PB_CR1 = 0xFF;
	PC_CR1 = 0xFF;
	PD_CR1 = 0xFF;
	
	// LED is push-pull output (CR1 already set)
	OUTPUT(LED_DEBUG);

	// Inputs have already pull-up resistors, so we turn pull-up
	// off (CR1) and enable interrupts (CR2).
	REG_LOW(CR1, SW_AWAY);
	REG_LOW(CR1, SW_HOME);
	REG_LOW(CR1, SW_BAT);
	REG_HIGH(CR2, SW_AWAY);
	REG_HIGH(CR2, SW_HOME);
	REG_HIGH(CR2, SW_BAT);

	// MOSFET controls and LEDs are push-pull outputs (CR1 already
	// set)
	OUTPUT(CTRL_PRI);
	OUTPUT(CTRL_HOME1);
	OUTPUT(CTRL_HOME2);
	OUTPUT(LED_AWAY);
	OUTPUT(LED_HOME);
	OUTPUT(LED_THIRD);

	// Interrupts on rising/falling edge on PORTC. TODO change
	// this if inputs are somewhere else than PORTC
	EXTI_CR1 |= 0x30;

	// Timer configuration
	// Prescaler register: 2MHz/2^4 = 125 kHz
	TIM2_PSCR = 4; // 
	// Counter Auto-Reload Registers. TIM2_ARR = 125, 1 millisecond
	TIM2_ARRH = 0;
	TIM2_ARRL = 125;
	// Interrupt Enable Register, Update interrupt (UIE)
	TIM2_IER |= TIM_IER_UIE;
	// Control Register 1, Counter ENable bit (CEN)
	TIM2_CR1 |= TIM_CR1_CEN;

	// Enabling interrupts should be the last part of
	// initialization.
	rim();

	// Stay in light sleep to keep timers going (unless halted in interrupt)
	while (true) {
		wfi();
	}
}
