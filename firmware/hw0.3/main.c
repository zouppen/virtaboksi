/* Firmware for Virtaboksi v0.3 */
#include <stdint.h>
#include <stdbool.h>
#include "stm8.h"

// Pin configuration
#define LED_PORT          PD
#define LED_PIN           PIN4
#define SW_OFF_PORT       PC
#define SW_OFF_PIN        PIN5
#define SW_BAT_PORT       PC
#define SW_BAT_PIN        PIN4
#define SW_HOME_PORT      PC
#define SW_HOME_PIN       PIN3
#define CTRL_HOME24_PORT  PD
#define CTRL_HOME24_PIN   PIN2
#define CTRL_HOME12_PORT  PC
#define CTRL_HOME12_PIN   PIN7
#define CTRL_PRI_PORT     PD
#define CTRL_PRI_PIN      PIN3

// On bootup, do control immediately, but not too immediately to avoid
// oscillation in case of a boot loop.
static volatile uint16_t ctrl_debounce = 500;

void update_outputs(void);

void update_outputs() {
	bool sw_off = !(PORT(SW_OFF_PORT, IDR) &= SW_OFF_PIN);
	bool sw_bat = !(PORT(SW_BAT_PORT, IDR) &= SW_BAT_PIN);
	bool sw_home = !(PORT(SW_HOME_PORT, IDR) &= SW_HOME_PIN);

	if (sw_off || sw_bat) {
		// BAT bad or OFF state: all outputs off
		PORT(CTRL_PRI_PORT, ODR) &= ~CTRL_PRI_PIN;
		PORT(CTRL_HOME24_PORT, ODR) &= ~CTRL_HOME24_PIN;
		PORT(CTRL_HOME12_PORT, ODR) &= ~CTRL_HOME12_PIN;
	} else if (sw_home) {
		// Home: All outputs on
		PORT(CTRL_PRI_PORT, ODR) |= CTRL_PRI_PIN;
		PORT(CTRL_HOME24_PORT, ODR) |= CTRL_HOME24_PIN;
		PORT(CTRL_HOME12_PORT, ODR) |= CTRL_HOME12_PIN;
	} else {
		// Away: PRI on
		PORT(CTRL_PRI_PORT, ODR) |= CTRL_PRI_PIN;
		PORT(CTRL_HOME24_PORT, ODR) &= ~CTRL_HOME24_PIN;
		PORT(CTRL_HOME12_PORT, ODR) &= ~CTRL_HOME12_PIN;
	}
}

void debounce_start(void) __interrupt(EXTI3_IRQ) {
	ctrl_debounce = 1000;
}

void debounce_check(void) __interrupt(TIM2_OVR_UIF_IRQ) {
	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;
	
	// Blink the LED when in operation
	PORT(LED_PORT, ODR) ^= LED_PIN;

	if (ctrl_debounce) {
		if (!--ctrl_debounce) {
			// Indicator LED turns off
			PORT(LED_PORT, ODR) &= ~LED_PIN;

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
	// to pull-up state. We make them all pull-up first and then
	// set outputs.
	PORT(PA, CR1) = 0xFF;
	PORT(PB, CR1) = 0xFF;
	PORT(PC, CR1) = 0xFF;
	PORT(PD, CR1) = 0xFF;
	
	// LED is push-pull output (CR1 already set)
	PORT(LED_PORT, DDR) |= LED_PIN;

	// Switches are inputs with interrupts and pull-up (CR1 already set)
	PORT(SW_OFF_PORT, CR2) |= SW_OFF_PIN;
	PORT(SW_BAT_PORT, CR2) |= SW_BAT_PIN;
	PORT(SW_HOME_PORT, CR2) |= SW_HOME_PIN;

	// MOSFET controls are push-pull outputs (CR1 already set)
	PORT(CTRL_PRI_PORT, DDR) |= CTRL_PRI_PIN;
	PORT(CTRL_HOME24_PORT, DDR) |= CTRL_HOME24_PIN;
	PORT(CTRL_HOME12_PORT, DDR) |= CTRL_HOME12_PIN;

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
	enableInterrupts();

	// Stay in light sleep to keep timers going (unless halted in interrupt)
	while (true) {
		wfi();
	}
}
