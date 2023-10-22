/* Firmware for Virtaboksi v0.4 and above */
#include <stm8.h>
#include "board.h"
#include "iref.h"
#include "util.h"

#define DEBOUNCE_MS 200
#define STARTUP_DEBOUNCE_MS 500
#define MINIMUM_WAKEUP_MS 100
#define SERIAL_KEEPALIVE_MS 1000

// On bootup, have a small pause after bootup before switching loads,
// to avoid oscillation in case of a boot loop.
static volatile uint16_t ctrl_debounce = STARTUP_DEBOUNCE_MS;

// Used to postpone sleeping for any reason, including waiting for
// serial traffic.
static volatile uint16_t snooze_suppressor = 0;

// A global flag for carrying state from ISR to main loop
static volatile bool timers_running = true;

// These are handled inside ISRs only so no need to make them
// volatile.
#define SERIAL_BUF_SIZE 80
static uint8_t serial_buf[SERIAL_BUF_SIZE];
static uint8_t *serial_rx_p = serial_buf;
static const uint8_t *serial_tx_p = serial_buf;
static void *serial_tx_end; // Indicator when to stop sending

static void controlled_halt(void);
static void update_outputs(void);
static void loop(void);

// Halts CPU. This must be called outside of interrupt handlers.
static void controlled_halt(void)
{
	// Must not go to interrupts while preparing halt()
	sim();

	// "Unreal mode" for serial traffic. Enable interrupt for the
	// serial pin and it magically wakes up after serial activity,
	// although this shouldn't be possible according to STM8S data
	// sheet (rx can't emit an interrupt while in HALT mode.
	REG_HIGH(CR2, PIN_RX);

	// Turn off the running indicator
	LOW(PIN_LED_PCB);

	// We may wake up because of start bit on UART line, meaning
	// the first byte hasn't been yet received. Ensuring that
	// we'll stay awake until the end of first byte at minimum.
	snooze_suppressor = MINIMUM_WAKEUP_MS;
	timers_running = true;

	// Put IREF to powersave if no LEDs are lit
	iref_maybe_off();

	halt();

	// WAKE UP!! Rise and shine! Interrupts are automatically
	// enabled on wake-up, so suppressing them again. In case of
	// serial traffic, int_on_portd() has been already called
	// before getting here because it's impossible to wake up
	// without going to interrupt handlers.
	__critical {
		// Suppress future spurious calls to int_on_portd()
		REG_LOW(CR2, PIN_RX);

		// Put IREF on while we stay awake
		iref_on();
	}
}

static void update_outputs(void)
{
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
		LOW(PIN_OUT1);
		LOW(PIN_OUT2);
	} else if (sw_home) {
		// Home: All outputs on
		HIGH(PIN_GROUP1);
		HIGH(PIN_GROUP2);
		HIGH(PIN_GROUP3);
		// Indicator LEDs
		LOW(PIN_OUT1);
		HIGH(PIN_OUT2);
	} else {
		// Away: PRI on
		HIGH(PIN_GROUP1);
		LOW(PIN_GROUP2);
		LOW(PIN_GROUP3);
		// Indicator LEDs
	        HIGH(PIN_OUT1);
		LOW(PIN_OUT2);
	}
}

void uart_rx(void) __interrupt(UART1_RX)
{
	// Incoming data
	if (UART1_SR & UART_SR_RXNE) {
		// Reading the byte also clears the RXNE flag
		uint8_t const chr = UART1_DR;

		// Keep CPU running until we've received a whole message
		snooze_suppressor = SERIAL_KEEPALIVE_MS;

		// Funny little test
		if (chr == '\r') {
			// Terminate string
			*serial_rx_p++ = '\r';
			*serial_rx_p++ = '\n';

			// Reset buffers
			serial_tx_end = serial_rx_p;
			serial_tx_p = serial_buf;
			serial_rx_p = serial_buf;

			// Enable RS-485
			HIGH(PIN_TX_EN);

			// Enable interrupt to suck the data from tx buffer
			UART1_CR2 |= UART_CR2_TIEN;
		} else if (serial_rx_p == serial_buf + SERIAL_BUF_SIZE - 2) {
			// Buffer overflow. Ignore byte.
		} else {
			// Have some fun by running rot13 on input
			*serial_rx_p++ = util_rot13(chr);
		}
	}
}

void uart_tx(void) __interrupt(UART1_TX)
{
	// Since we share the same interrupt, we have to have a state
	// for the send and finish conditions.
	if (serial_tx_p < serial_tx_end) {
		// Can we yet send?
		if (UART1_SR & UART_SR_TXE) {
			// Check if want to transmit more
			uint8_t const out = *serial_tx_p++;
			if (serial_tx_p == serial_tx_end) {
				// Do not come here again and wait enable the
				// TX ready interrupt.
				UART1_CR2 ^= UART_CR2_TIEN | UART_CR2_TCIEN;
			}

			// Send byte
			UART1_DR = out;
		}
	} else {
		// Is the last byte already finished
		if (UART1_SR & UART_SR_TC) {
			// Transmit finished. Disable TCIEN
			UART1_CR2 &= ~UART_CR2_TCIEN;

			// Turn off RS-485 transmitter
			LOW(PIN_TX_EN);
		}
	}
}

void int_on_portc(void) __interrupt(EXTI2_IRQ)
{
	ctrl_debounce = DEBOUNCE_MS;
}

void int_on_portd(void) __interrupt(EXTI3_IRQ)
{
	// OBS! When we wake up from serial activity, we get here once
	// and it's inevitable.
}

void run_every_1ms(void) __interrupt(TIM2_OVR_UIF_IRQ)
{
	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;

	// Blink the LED when in operation
	TOGGLE(PIN_LED_PCB);

	// This is an optimization, can keep in register until end of
	// this function.
	bool running = false;

	// Debounce countdown
	if (ctrl_debounce) {
		if (--ctrl_debounce) {
			running = true;
		} else {
			update_outputs();
		}
	}

	// Serial runtime counter
	if (snooze_suppressor) {
		if (--snooze_suppressor) {
			running = true;
		}
	}

	// Update the global flag to indicate readiness for a sleep.
	timers_running = running;
}

int main(void)
{
	// Setting clock speed
	stm8_configure_clock();

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

	// Serial port has an external pull-up, no need to put CR1 on.
	REG_LOW(CR1, PIN_RX); // We have external pull-up

	// MOSFET controls and LEDs are push-pull outputs
	// (CR1 already set)
	OUTPUT(PIN_GROUP1);
	OUTPUT(PIN_GROUP2);
	OUTPUT(PIN_GROUP3);
	OUTPUT(PIN_OUT1);
	OUTPUT(PIN_OUT2);
	OUTPUT(PIN_OUT3);

	// Must be on before LEDs are controlled
	BOARD_IREF_CONF;
	iref_on();

	// Interrupts are board specific and defined in board.h.
	EXTI_CR1 = BOARD_EXTI_CR1;

	// Timer configuration
	// Prescaler register: 2MHz/2^4 = 125 kHz
	TIM2_PSCR = 4;
	// Counter Auto-Reload Registers. TIM2_ARR = 125, 1 millisecond
	TIM2_ARRH = 0;
	TIM2_ARRL = 125;
	// Interrupt Enable Register, Update interrupt (UIE)
	TIM2_IER = TIM_IER_UIE;
	// Control Register 1, Counter ENable bit (CEN)
	TIM2_CR1 = TIM_CR1_CEN;

	// UART configuration
	UART1_CR2 =
		UART_CR2_TEN |   // Enable TX
		UART_CR2_REN |   // Receiver enable
		UART_CR2_RIEN;   // Receiver interrupt enabled
	// UART1_CR3 default is 1 stop bit
	stm8_uart1_baudrate(9600);

	// Turn on rs485 rx and put to receive mode
	OUTPUT(PIN_TX_EN);
	LOW(PIN_TX_EN);

	// Enabling interrupts should be the last part of
	// initialization.
	rim();

	// Stay in light sleep to keep timers going. If nothing to
	// run, go to halt mode.
	while (true) {
		loop();

		if (!timers_running && !STATE(PIN_TX_EN)) {
			controlled_halt();
		} else {
			wfi();
		}
	}
}

// Run after waking up from the interrupts. Can be used to
// perform longer duration tasks. Interrupts are enabled.
static void loop(void)
{
}
