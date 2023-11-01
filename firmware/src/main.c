/* Firmware for Virtaboksi v0.4 and above */
#include <stm8.h>
#include <string.h>
#include "board.h"
#include "iref.h"
#include "util.h"
#include "serial.h"
#include "timers.h"
#include "settings.h"

#define HALT_ENABLED

// On bootup, have a small pause after bootup before switching loads,
// to avoid oscillation in case of a boot loop.
static uint16_t ctrl_debounce = STARTUP_DEBOUNCE_MS;

// In case excessive flipflopping, turn all outputs off since it might
// be an indicator of a hardware error.
static uint16_t ctrl_panic = PANIC_OFF_MS;

// A global flag for carrying state from ISR to main loop
static volatile bool timers_running = true;

static void controlled_halt(void);
static void update_outputs_IM(bool const panic);
static void debounce_arm_IM(void);
static bool debounce_tick_IM(void);
static void setup_IM(void);
static void loop(void);

// Halts CPU. This must be called outside of interrupt handlers but
// interrupts disabled. Like wfi(), enables interrupts on return.
static void controlled_halt(void)
{
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
	timers_stay_awake_IM(MINIMUM_WAKEUP_MS);
	timers_running = true;

	// Put IREF to powersave if no LEDs are lit
	iref_maybe_off_IM();

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
		iref_on_IM();
	}
}

static void update_outputs_IM(bool const panic)
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

static void debounce_arm_IM(void)
{
	ctrl_debounce = DEBOUNCE_MS;
}

static bool debounce_tick_IM(void)
{
	if (ctrl_debounce) {
		if (--ctrl_debounce) {
			// Still running
			if (ctrl_panic && !--ctrl_panic) {
				// Running for too long
				update_outputs_IM(true);
			}
			return true;
		}
		// Reached zero
		ctrl_panic = PANIC_OFF_MS;
		update_outputs_IM(false);
	}
	return false;
}

void int_on_portb(void) __interrupt(EXTI1_ISR)
{
	debounce_arm_IM();
}

void int_on_portc(void) __interrupt(EXTI2_ISR)
{
	debounce_arm_IM();
}

void int_on_portd(void) __interrupt(EXTI3_ISR)
{
	// OBS! When we wake up from serial activity, we get here once
	// and it's inevitable.
}

void run_every_1ms(void) __interrupt(TIM2_OVF_ISR)
{
	// Clear Timer 2 Status Register 1 Update Interrupt Flag (UIF)
	TIM2_SR1 &= ~TIM_SR1_UIF;

	// Blink the LED when in operation
	TOGGLE(PIN_LED_PCB);

	// Process ticks
	bool const debouncing = debounce_tick_IM();
	bool const ticking = timers_tick_IM();
	serial_tick_IM();

	// Update the global flag to indicate readiness for a sleep.
	timers_running = debouncing || ticking;
}

int main(void)
{
	// Configure device and enable interrupts.
	setup_IM();
	enable_interrupts();

	// Stay in light sleep to keep timers going. If nothing to
	// run, go to halt mode.
	while (true) {
		loop();

		// Block interrupts while whe figure out sleep method.
		disable_interrupts();

		// If there are still messages to process, go to loop.
		if (serial_has_message_IM()) {
			enable_interrupts();
			continue;
		}
#ifdef HALT_ENABLED
		if (!timers_running && !serial_is_transmitting()) {
			controlled_halt();
		} else {
			wfi();
		}
#else
		wfi();
#endif
		// Both controlled_halt() and wfi() implicitly enable
		// interrupts.
	}
}

static void setup_IM(void)
{
	// Setting clock speed
	stm8_configure_clock();

	settings_init();

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
	iref_on_IM();

	// Make interrupts trigger on both edges
	EXTI_CR1 = 0xff;

	// Timer prescale to 125 kHz
	TIM2_PSCR = TICK_PRESCALE;
	// Counter Auto-Reload Registers. TIM2_ARR = 125, 1 millisecond
	TIM2_ARRH = 0;
	TIM2_ARRL = 125;
	// Interrupt Enable Register, Update interrupt (UIE)
	TIM2_IER = TIM_IER_UIE;
	// Control Register 1, Counter ENable bit (CEN)
	TIM2_CR1 = TIM_CR1_CEN;

	serial_init(settings.baud_rate);
}

// Run after waking up from the interrupts. Can be used to
// perform longer duration tasks. Interrupts are enabled.
static void loop(void)
{
	serial_buffer_t *const rx = serial_get_message();
	if (rx == NULL) return;

	if (rx->state == TEXT) {
		if (!strcmp(rx->data, "PAUSES")) {
			strcpy(serial_tx, "taukoja ei ole");
		} else {
			// We've got a line, let's convert it. This is a
			// little bit unsafe, but is a placeholder for the
			// real logic only.
			strcpy(serial_tx, "Saatiin: ");
			buflen_t const head = 9;
			for (buflen_t i=0; i<SERIAL_TX_LEN-head; i++) {
				char const c = rx->data[i];
				if (c == '\0') {
					serial_tx[head+i] = '\0';
					break;
				} else {
					serial_tx[head+i] = util_rot13(c);
				}
			}
		}
		serial_tx_line();
	} else {
		// We don't understand this message type (yet)
	}

	serial_free_message();
}
