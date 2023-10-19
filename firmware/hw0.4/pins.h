#pragma once

// Board version 0.4 configuration
#ifdef BOARD_0_4
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
#define PIN_TX_EN     PD,2,3

// Interruptable pins are on PORTC and PORTB
#define BOARD_EXTI_CR1 (EXTI_CR1_C_BOTH | EXTI_CR1_D_BOTH)
#endif

// Constant pins for STM8S003
#define PIN_RX        PD,6
