#pragma once

// Binary safe RS-485 interface

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t buflen_t;

// Serial buffer lengths. If you want to go beyond 255, remember to
// change buflen_t from uint8_t to uint16_t.
#define SERIAL_RX_LEN 80
#define SERIAL_TX_LEN 80

// Useful return value. Use < SERIAL_TX_LEN in comparison instead of
// this because this indicates the maximum value.
#define BUFLEN_MAX ({ buflen_t _a = ~0; _a; })

typedef enum {
	EMPTY,    // No message, p is undefined
	BINARY,   // Message contains binary payload
	TEXT,     // Message is NUL-terminated ASCII
	BREAK,    // Message is interrupted by a BREAK
	OVERFLOW  // Message has overflown
} serial_state_t;

typedef struct {
	int len;                      // Buffer length
	serial_state_t state;         // Buffer state
	uint8_t data[SERIAL_RX_LEN];  // Pointer to buffer
} serial_buffer_t;

typedef struct {
	int flip_timeout; // Number of missed flips
	int too_long_tx;  // Number of too long frames in transmit
} serial_counter_t;

// Serial buffer which is sent on invocation of serial_tx_start().
extern char serial_tx[SERIAL_TX_LEN];

// Initialize serial port.
void serial_init(void);

// Is serial transmitter on?
bool serial_is_transmitting(void);

// Does serial have message waiting? Must be called with interrupts
// disabled!
bool serial_has_message_IM(void);

// Called from timer interrupt regularily
void serial_tick_IM(void);

// Gets a message from serial receive buffer. Buffer must be released
// after processing with serial_free_message(). In case no message is
// received, NULL is returned
serial_buffer_t *serial_get_message(void);

// Release receive buffer. This is important to do as soon as
// possible. If frame is not freed before back buffer is filled,
// error_flip_timeout occurs.
void serial_free_message(void);

// Start half-duplex transmission (disables rx). For sending NUL
// terminated strings. Outputs newline after every send.
void serial_tx_line(void);

// Start half-duplex transmission (disables rx). Binary safe.
void serial_tx_bin(buflen_t const len);

// Get serial counters and zero them.
void serial_pull_counters(serial_counter_t *copy);

// UART interrupts
void serial_int_uart_rx(void) __interrupt(UART1_RXC_ISR);
void serial_int_uart_tx(void) __interrupt(UART1_TXC_ISR);
