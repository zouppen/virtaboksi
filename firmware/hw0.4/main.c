/* Firmware for Virtaboksi v0.4 and above */
#include <stm8.h>
#include <string.h>
#include "board.h"
#include "iref.h"
#include "util.h"
#include "serial.h"

#define DEBOUNCE_MS 200
#define STARTUP_DEBOUNCE_MS 500
#define PANIC_OFF_MS 1000
#define MINIMUM_WAKEUP_MS 100
#define SERIAL_KEEPALIVE_MS 1000
#define HALT_ENABLED

// On bootup, have a small pause after bootup before switching loads,
// to avoid oscillation in case of a boot loop.
static volatile uint16_t ctrl_debounce = STARTUP_DEBOUNCE_MS;

// In case excessive flipflopping, turn all outputs off since it might
// be an indicator of a hardware error.
static volatile uint16_t ctrl_panic = PANIC_OFF_MS;

// Used to postpone sleeping for any reason, including waiting for
// serial traffic.
static volatile uint16_t snooze_suppressor = 0;

// A global flag for carrying state from ISR to main loop
static volatile bool timers_running = true;

static volatile bool end_of_line = false;

// These are handled inside ISRs only so no need to make them
// volatile.
static uint8_t serial_rx[SERIAL_RX_LEN];
static uint8_t *serial_rx_p = serial_rx;

static void controlled_halt(void);
static void update_outputs(bool const panic);
static void debounce_arm(void);
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

static void update_outputs(bool const panic)
{
	bool const sw_away = !READ(PIN_IN1);
	bool const sw_home = !READ(PIN_IN2);
	bool const bat_good = READ(PIN_IN3);
	bool const sw_main = sw_home || sw_away;

	if (panic || !bat_good || !sw_main) {
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

static void debounce_arm(void)
{
	ctrl_debounce = DEBOUNCE_MS;
}

void uart_rx(void) __interrupt(UART1_RX)
{
	// Cache values to avoid the register getting cleared. This
	// sequence also clears register values.
	uint8_t const sr = UART1_SR;
	uint8_t const chr = UART1_DR;

	if (sr & UART_SR_FE) {
		// Break character or garbage. Ignore.
	} else if (sr & UART_SR_RXNE) {
		// Incoming proper data

		// Keep CPU running until we've received a whole message
		snooze_suppressor = SERIAL_KEEPALIVE_MS;

		serial_rx_activity();

		if (end_of_line) {
			// End of line already reached
		} else if (serial_rx_p == serial_rx + SERIAL_RX_LEN) {
			// Buffer overflow imminent!
		} else {
			// Okay, getting byte then
			*serial_rx_p++ = chr;
		}

		if (chr == '\n') {
			end_of_line = true;
		}
	}
}

void int_on_portb(void) __interrupt(EXTI1_IRQ)
{
	debounce_arm();
}

void int_on_portc(void) __interrupt(EXTI2_IRQ)
{
	debounce_arm();
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

			// Panic condition countdown
			if (ctrl_panic) {
				if (!--ctrl_panic) {
					update_outputs(true);
				}
			}
		} else {
			// Input is settled
			ctrl_panic = PANIC_OFF_MS;
			update_outputs(false);
		}
	}

	// Serial runtime counter
	if (snooze_suppressor) {
		if (--snooze_suppressor) {
			running = true;
		}
	}

	serial_tick();

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
	PE_CR1 = 0xFF;
	PF_CR1 = 0xFF;

	// LED is push-pull output (CR1 already set)
	OUTPUT(PIN_LED_PCB);

	// Inputs have already discrete pull-up resistors, so we turn
	// their pull-up off (CR1) and enable interrupts (CR2).
	REG_LOW(CR1, PIN_IN1);
	REG_LOW(CR1, PIN_IN2);
	REG_LOW(CR1, PIN_IN3);
	REG_LOW(CR1, PIN_IN4);
	REG_HIGH(CR2, PIN_IN1);
	REG_HIGH(CR2, PIN_IN2);
	REG_HIGH(CR2, PIN_IN3);
	REG_HIGH(CR2, PIN_IN4);

	// Serial port has an external pull-up, too, disabling pull-up
	REG_LOW(CR1, PIN_RX);

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

	// Make interrupts trigger on both edges
	EXTI_CR1 = 0xff;

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

	serial_init();

	// Enabling interrupts should be the last part of
	// initialization.
	rim();

	// Stay in light sleep to keep timers going. If nothing to
	// run, go to halt mode.
	while (true) {
		loop();

#ifdef HALT_ENABLED
		if (!timers_running && !serial_is_transmitting()) {
			controlled_halt();
		} else {
			wfi();
		}
#else
		wfi();
#endif
	}
}

// Run after waking up from the interrupts. Can be used to
// perform longer duration tasks. Interrupts are enabled.
static void loop(void)
{
	if (end_of_line) {
		// We've got a line, let's convert it. This is a
		// little bit unsafe, but is a placeholder for the
		// real logic only.
		strcpy(serial_tx, "Saatiin: ");
		buflen_t const head = 9;
		for (buflen_t i=0; i<SERIAL_TX_LEN-head; i++) {
			char const c = serial_rx[i];
			if (c == '\n') {
				serial_tx[head+i] = '\0';
				break;
			} else {
				serial_tx[head+i] = util_rot13(c);
			}
		}
		serial_rx_p = serial_rx;
		end_of_line = false;
		serial_tx_line();
	}
}
