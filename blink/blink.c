/* The "Hello world!" of microcontrollers. Blink LED on/off */
#include <stdint.h>
#include <stdbool.h>
#include "stm8.h"


#define LED_PORT     PA
#define LED_PIN      PIN3
#define SW_OFF_PORT  PC
#define SW_OFF_PIN   PIN7
#define SW_BAT_PORT  PC
#define SW_BAT_PIN   PIN6
#define SW_HOME_PORT PC
#define SW_HOME_PIN  PIN5

static uint16_t off_debounce = 0;

void debounce_start(void) __interrupt(EXTI3_IRQ) {
	off_debounce = 10000;
}

void debounce_check(void) __interrupt(TIM2_OVR_UIF_IRQ) {
	bool any_running = false;

	// Hack
	PORT(LED_PORT, ODR) ^= LED_PIN; // PB_ODR |= (1 << 5);
	
	if (off_debounce) {
		if (!--off_debounce) {
			// Reached zero, set to target state
			if (PORT(SW_OFF_PORT, IDR) &= SW_OFF_PIN) {
				PORT(LED_PORT, ODR) |= LED_PIN;
			} else {
				PORT(LED_PORT, ODR) &= ~LED_PIN;
			}
		} else {
			any_running = true;
		}	
	}

	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;

	if (!any_running) {
		// All timers in idle, halt the whole CPU
		halt();
	}
}


int main(void)
{
	// Not changing clock speed, running on default of 2 MHz

	// LED is output, push-pull
	PORT(LED_PORT, DDR) |= LED_PIN;
	PORT(LED_PORT, CR1) |= LED_PIN;

	// Switches are inputs with pull-ups and interrupts
	PORT(SW_OFF_PORT, CR1) |= SW_OFF_PIN;
	PORT(SW_OFF_PORT, CR2) |= SW_OFF_PIN;
	
	PORT(SW_BAT_PORT, CR1) |= SW_BAT_PIN;
	PORT(SW_BAT_PORT, CR2) |= SW_BAT_PIN;
	
	PORT(SW_HOME_PORT, CR1) |= SW_HOME_PIN;
	PORT(SW_HOME_PORT, CR2) |= SW_HOME_PIN;

	// Interrupts on rising/falling edge on PORTC
	EXTI_CR1 |= 0x30;

	// Timer configuration
	// Prescaler register
	TIM2_PSCR = 4; // 2MHz/2^4 = 125 kHz
	// Set Counter Auto-Reload Registers - TIM2_ARR=125, 1 millisecond
	TIM2_ARRH = 0;
	TIM2_ARRL = 125;
	// Interrupt Enable Register, Update interrupt (UIE)
	TIM2_IER |= TIM_IER_UIE;
	// Control Register 1, Counter ENable bit (CEN)
	TIM2_CR1 |= TIM_CR1_CEN;

	enableInterrupts();

	while(1) {
		// Stay on light sleep mode
		wfi();
	}
}
