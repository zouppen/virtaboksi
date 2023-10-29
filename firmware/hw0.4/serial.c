#include <stdlib.h>
#include <stdint.h>
#include <stm8.h>
#include <string.h>
#include "board.h"
#include "serial.h"
#include "timers.h"

#define NUL_TERMINATED 0

// Serial transmit vars
char serial_tx[SERIAL_TX_LEN]; // Outgoing serial data
static const uint8_t *serial_tx_p = serial_tx; // Pointer to tx position
static volatile void *serial_tx_end; // Indicator when to stop sending
static volatile bool tx_state = false; // Is tx start requested
static volatile bool tx_nul_terminated; // Flag for \0 terminated buffer

// Serial receive vars. Using double buffering for rx
static char serial_rx_a[SERIAL_RX_LEN]; // Receive buffer a
static char serial_rx_b[SERIAL_RX_LEN]; // Receive buffer b
static char *serial_rx_back = serial_rx_a; // Back buffer (for populating data)
static char *serial_rx_p = serial_rx_a; // Back buffer write pointer
static char *serial_rx_front = NULL; // Contains front buffer if it's not yet freed
static buflen_t serial_rx_front_len; // Contains front buffer data length
static volatile uint16_t rx_cooldown_left = 0; // To avoid half-duplex clash

// (Error) counters
static volatile serial_counter_t counts = {0, 0, 0};

// Prototypes
static void end_of_frame(bool const good);
static void may_send(buflen_t len);
static void transmit_now(void);

// Called from timer interrupt handler when we're between frames.
static void end_of_frame(bool const good)
{
	bool locked = serial_rx_front != NULL;
	if (locked) {
		// We need to throw a frame overboard
		// because main loop didn't process
		// front buffer in time.
		counts.flip_timeout++;
		goto rewind;
	}
	if (good) {
		// Looks good, at least before CRC checks and so.
		counts.good++;
	} else {
		// Too long frame increments error counter. We still want to sometimes process the start of it
		counts.too_long_rx++;
	}

	// Collect length for later use.
	serial_rx_front_len = serial_rx_p == NULL
		? SERIAL_RX_LEN
		: serial_rx_p - serial_rx_back;

	// Flip buffers!
	serial_rx_front = serial_rx_back;
	serial_rx_back = (serial_rx_a == serial_rx_back)
		? serial_rx_b
		: serial_rx_a;

rewind:
	// Latest but not least: Rewind receive buffer
	serial_rx_p = serial_rx_back;
}

buflen_t serial_get_message(char **const buf) __critical
{
	int len;
	if (serial_rx_front == NULL) {
		*buf = NULL;
		return 0;
	}

	*buf = serial_rx_front;
	len = serial_rx_front_len;
	return len;
}

void serial_free_message(void) __critical
{
	serial_rx_front = NULL;
}

void pull_serial_counters(serial_counter_t *const copy) __critical
{
	memcpy(copy, &counts, sizeof(counts));
	memset(&counts, 0, sizeof(counts));
}

void serial_init(void)
{
	// UART configuration
	UART1_CR2 =
		UART_CR2_TEN |   // Enable TX
		UART_CR2_REN |   // Receiver enable
		UART_CR2_RIEN;   // Receiver interrupt enabled
	// UART1_CR3 default is 1 stop bit
	stm8_uart1_baudrate(BAUD);

	// Turn on rs485 rx and put to receive mode
	OUTPUT(PIN_TX_EN);
	LOW(PIN_TX_EN);
}

bool serial_is_transmitting(void)
{
	return tx_state;
}

void serial_tx_line(void)
{
	// Putting the continuation sign to the end unless it's
	// exactly that long message
	char *const last = serial_tx + SERIAL_TX_LEN - 1;
	if (*last != '\0') {
		*last = '>';
	}

	// Sending until NUL byte, maximum of buffer length
	may_send(NUL_TERMINATED);
}

void serial_tx_bin(buflen_t const len)
{
	// The buffer is allowed to be completely full, that's why
	// comparing greater than instead of greater-than-equals.
	if (len > SERIAL_TX_LEN) {
		// Ignore the whole frame if it's too long.
		__critical {
			counts.too_long_tx++;
		}
		return;
	}

	if (len == 0) {
		// Must not go to transmit phase at all.
		return;
	}

	// Use supplied length
	may_send(len);
}

static void may_send(buflen_t len) __critical
{
	// Setting end pointer accordingly
	if (len == NUL_TERMINATED) {
		// This is a special case, end pointer may be over one
		// char since when it's last char, the line is
		// terminated anyway. This causes one read over buffer
		// but nothing fatal. See UART1_TX interrupt.
		tx_nul_terminated = true;
		serial_tx_end = serial_tx + SERIAL_TX_LEN + 1;
	} else {
		tx_nul_terminated = false;
		serial_tx_end = serial_tx + len;
	}

	// Setting a flag of willingness to perform a transmission
	tx_state = true;

	// RS-485 is half duplex. We need to wait until rx line
	// becomes idle. If it's not, serial_tick calls transmit_now()
	// later.
	if (!rx_cooldown_left) {
		transmit_now();
	}
}

void serial_tick(void) {
	if (!rx_cooldown_left) {
		// Timer has already reached zero
		return;
	}
	if (!--rx_cooldown_left && tx_state) {
		// Timer reached zero and we have something to send.
		transmit_now();
	}
}

static void transmit_now(void)
{
	// Enable RS-485
	HIGH(PIN_TX_EN);

	// Let the TX interrupt to run
	UART1_CR2 |= UART_CR2_TIEN;
}

void serial_int_uart_rx(void) __interrupt(UART1_RX)
{
	// Cache values to avoid the register getting cleared. This
	// sequence also clears register values.
	uint8_t const sr = UART1_SR;
	uint8_t const chr = UART1_DR;

	if (sr & UART_SR_FE) {
		// BREAK signal or garbage.
		serial_rx_p = serial_rx_back;
	} else if (sr & UART_SR_RXNE) {
		// Incoming proper data

		// Keep CPU running until we've received a whole message
		timers_stay_awake(SERIAL_KEEPALIVE_MS);

		// Resetting rx timers
		rx_cooldown_left = MODBUS_SILENCE;

		bool const full = serial_rx_p == serial_rx_back + SERIAL_RX_LEN;

		if (serial_rx_p == NULL) {
			// Overflow has already happened earlier.
			if (chr == '\n') {
				end_of_frame(false);
			}
		} else if (chr == '\n') {
			// End of line
			if (!full) {
				// Terminate string
				*serial_rx_p = '\0';
			}
			end_of_frame(true);
		} else if (full) {
			// Overflow happens now but is not line terminator
			serial_rx_p = NULL;
		} else {
			// We have space for a new char
			*serial_rx_p++ = chr;
		}
	}
}

void serial_int_uart_tx(void) __interrupt(UART1_TX)
{
	// Since we share the same interrupt, we have to have check if
	// we have enabled such event before going to check the flags.

	// Check if we are ready to send
	if ((UART1_CR2 & UART_CR2_TIEN) && (UART1_SR & UART_SR_TXE)) {
		uint8_t const out = *serial_tx_p++;

		// Check if we want to transmit more
		bool const buf_end = serial_tx_p == serial_tx_end;
		bool const line_end = tx_nul_terminated && (buf_end || out == '\0');
		if (buf_end || line_end) {
			// This is the last byte. Disable this event
			// and enable rx ready event
			UART1_CR2 ^= UART_CR2_TIEN | UART_CR2_TCIEN;
		}

		// Send byte
		UART1_DR = line_end ? '\n' : out;
	}

	// Check if the transmit is finished
	if ((UART1_CR2 & UART_CR2_TCIEN) && (UART1_SR & UART_SR_TC)) {
		// Transmit finished. Disable this event
		UART1_CR2 &= ~UART_CR2_TCIEN;

		// Turn off RS-485 transmitter
		LOW(PIN_TX_EN);

		// Rewind send buffer
		serial_tx_p = serial_tx;

		// Ready for sleep or for a new send
		tx_state = false;
	}
}
