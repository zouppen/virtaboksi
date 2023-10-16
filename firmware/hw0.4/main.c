/* Firmware for Virtaboksi v0.4 */
#include <stdint.h>
#include <stdbool.h>
#include "stm8.h"

#define SERIAL_POWERDOWN_MS 10000

// Pin configuration
#define PIN_IN1       PC,7
#define PIN_IN2       PC,6
#define PIN_IN3       PC,5
#define PIN_IN4       PC,4
#define PIN_GROUP1    PD,4
#define PIN_GROUP2    PA,3
#define PIN_GROUP3    PA,2
#define PIN_OUT1      PB,4
#define PIN_OUT2      PB,5
#define PIN_OUT3      PC,3
#define PIN_LED_PCB   PD,1
#define PIN_TX_EN1    PD,2
#define PIN_TX_EN2    PD,3

// Non-configurable
#define PIN_RX        PD,6

// On bootup, have a small pause after bootup before switching loads,
// to avoid oscillation in case of a boot loop.
static volatile uint16_t ctrl_debounce = 500;

// Serial activity counter
static volatile uint16_t serial_runtime = 0;

bool uart1_baudrate(uint16_t rate);
void update_outputs(void);

bool uart1_baudrate(uint16_t rate) {
	uint32_t const freq = 2000000;
	uint32_t const div = freq/rate;

	if (div > 0xffff) return false;
	if (div < 16) return false;

	// Pick up nibbles and write in correct order (BRR2 first)
	UART1_BRR2 = ((div & 0xf000) >> 8) | (div & 0x000f);
	UART1_BRR1 = (div & 0x0ff0) >> 4;

	return true;
}

void update_outputs() {
	bool const sw_away = !READ(PIN_IN1);
	bool const sw_home = !READ(PIN_IN2);
	bool const bat_good = READ(PIN_IN3);
	bool const sw_main = sw_home || sw_away;

	if (!bat_good || !sw_main) {
		// BAT bad or OFF state: all outputs off
		LOW(PIN_GROUP1);
		LOW(PIN_GROUP2);
		LOW(PIN_GROUP3);
		// Indicator LEDs
		HIGH(PIN_OUT1);
		HIGH(PIN_OUT2);
		LOW(PIN_OUT3);
	} else if (sw_home) {
		// Home: All outputs on
		HIGH(PIN_GROUP1);
		HIGH(PIN_GROUP2);
		HIGH(PIN_GROUP3);
		// Indicator LEDs
		HIGH(PIN_OUT1);
		LOW(PIN_OUT2);
		HIGH(PIN_OUT3);
	} else {
		// Away: PRI on
		HIGH(PIN_GROUP1);
		LOW(PIN_GROUP2);
		LOW(PIN_GROUP3);
		// Indicator LEDs
		LOW(PIN_OUT1);
		HIGH(PIN_OUT2);
		HIGH(PIN_OUT3);
	}
}

void uart_rx(void) __interrupt(UART1_RX)
{
	// Incoming data
	if (UART1_SR & UART_SR_RXNE) {
		uint8_t const chr = UART1_DR;
		serial_runtime = SERIAL_POWERDOWN_MS;
	}
}

void int_on_portc(void) __interrupt(EXTI2_IRQ) {
	ctrl_debounce = 200;
}

void int_on_portd(void) __interrupt(EXTI3_IRQ) {
	// Have we woken up because of serial activity?
	if (!serial_runtime && READ(PIN_RX)) {
		// Receiver enable
		UART1_CR2 |= UART_CR2_REN;

		// Wait for data for 10 secs
		serial_runtime = SERIAL_POWERDOWN_MS;
	}
}

void run_every_1ms(void) __interrupt(TIM2_OVR_UIF_IRQ) {
	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;

	// Blink the LED when in operation
	TOGGLE(PIN_LED_PCB);

	bool sleepy = true;

	// Debounce countdown
	if (ctrl_debounce) {
		if (--ctrl_debounce) {
			sleepy = false;
		} else {
			update_outputs();
		}
	}

	// Serial runtime counter
	if (serial_runtime) {
		if (--serial_runtime) {
			sleepy = false;
		}
	}

	if (sleepy) {
		// Indicator LED turns off
		LOW(PIN_LED_PCB);

		// Shut down serial and prepare for a wakeup on start bit
		UART1_CR2 &= ~UART_CR2_REN;  // Receiver disable

		// Nothing more to count, so halt the whole CPU
		halt();
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
	OUTPUT(PIN_LED_PCB);

	// Inputs have already pull-up resistors, so we turn pull-up
	// off (CR1) and enable interrupts (CR2).
	REG_LOW(CR1, PIN_IN1);
	REG_LOW(CR1, PIN_IN2);
	REG_LOW(CR1, PIN_IN3);
	REG_LOW(CR1, PIN_IN4);
	REG_HIGH(CR2, PIN_IN1);
	REG_HIGH(CR2, PIN_IN2);
	REG_HIGH(CR2, PIN_IN3);
	REG_HIGH(CR2, PIN_IN4);

	// Serial traffic starts in standby mode
	REG_LOW(CR1, PIN_RX); // We have external pull-up
	REG_HIGH(CR2, PIN_RX); // Enable interrupts

	// MOSFET controls and LEDs are push-pull outputs
	// (CR1 already set)
	OUTPUT(PIN_GROUP1);
	OUTPUT(PIN_GROUP2);
	OUTPUT(PIN_GROUP3);
	OUTPUT(PIN_OUT1);
	OUTPUT(PIN_OUT2);
	OUTPUT(PIN_OUT3);

	// Interrupts on rising/falling edge on PORTC and PORTD. TODO
	// change this if inputs are somewhere else!
	EXTI_CR1 |= 0xf0;

	// Timer configuration
	// Prescaler register: 2MHz/2^4 = 125 kHz
	TIM2_PSCR = 4;
	// Counter Auto-Reload Registers. TIM2_ARR = 125, 1 millisecond
	TIM2_ARRH = 0;
	TIM2_ARRL = 125;
	// Interrupt Enable Register, Update interrupt (UIE)
	TIM2_IER |= TIM_IER_UIE;
	// Control Register 1, Counter ENable bit (CEN)
	TIM2_CR1 |= TIM_CR1_CEN;

	// UART configuration
	UART1_CR2 |=
		UART_CR2_TEN |  // Transmitter enable
		UART_CR2_RIEN;  // Receiver interrupt enabled
	UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
	uart1_baudrate(9600);

	// Turn on rs485 rx and put to receive mode
	OUTPUT(PIN_TX_EN1);
	OUTPUT(PIN_TX_EN2);
	LOW(PIN_TX_EN1);
	LOW(PIN_TX_EN2);

	// Enabling interrupts should be the last part of
	// initialization.
	rim();

	// Stay in light sleep to keep timers going (unless halted in interrupt)
	while (true) {
		wfi();
	}
}
