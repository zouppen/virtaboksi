/*
 * Register definitions for STM8S103 (and STM8S003)
 * Still incomplete.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Configure clock based on F_CPU.
void stm8_configure_clock(void);

// Set UART1 baud rate based on F_CPU. Returns false in case of an
// invalid value.
bool stm8_uart1_baudrate(uint32_t rate);

// Make EEPROM read-write
void stm8_eeprom_unlock(void);

// Make EEPROM read-only
void stm8_eeprom_lock(void);

// MACROS FOR EASY PIN HANDLING FOR ATMEL GCC-AVR, adapted to STM8
// From https://stackoverflow.com/a/25986570/514723

// Overloaded macro hack from
// https://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
#define _BV_GET_MACRO(_1,_2,NAME,...) NAME
#define _BV(...) _BV_GET_MACRO(__VA_ARGS__, _BV2, _BV1)(__VA_ARGS__)

// Macros for bit fields depending on bit count
#define _BV1(bit) (1U << (bit))
#define _BV2(bit_a, bit_b) ((1U << (bit_a)) | (1U << (bit_b)))

// These macros are used indirectly by other macros.
#define _SET(type,name,...)          name##_##type |= _BV(__VA_ARGS__)
#define _CLEAR(type,name,...)        name##_##type &= ~ _BV(__VA_ARGS__)
#define _TOGGLE(type,name,...)       name##_##type ^= _BV(__VA_ARGS__)
#define _GET(type,name,bit_a,...)    ((name##_##type >> bit_a) &  1)
#define _PUT(type,name,bit,value)    name##_##type = ( name##_##type & ( ~ _BV(bit)) ) | ( ( 1 & (unsigned char)value ) << bit )

// These macros are used by end user.
#define OUTPUT(...)         _SET(DDR,__VA_ARGS__)
#define INPUT(...)          _CLEAR(DDR,__VA_ARGS__)
#define HIGH(...)           _SET(ODR,__VA_ARGS__)
#define LOW(...)            _CLEAR(ODR,__VA_ARGS__)
#define TOGGLE(...)         _TOGGLE(ODR,__VA_ARGS__)
#define READ(...)           _GET(IDR,__VA_ARGS__)
#define STATE(...)          _GET(ODR,__VA_ARGS__)
#define REG_HIGH(reg,...)   _SET(reg,__VA_ARGS__)
#define REG_LOW(reg,...)    _CLEAR(reg,__VA_ARGS__)

// Shorthand for 8-bit registers
#define _SFR_(mem_addr)      (*(volatile uint8_t *)((mem_addr)))

// EEPROM address range
#define EEPROM_START_ADDR      0x4000
#define EEPROM_END_ADDR        0x407F

/* Register addresses */

/* Clock */
#define CLK_CKDIVR      _SFR_(0x50C6)
#define CLK_ICKR        _SFR_(0x50C0)
#define CLK_SWR         _SFR_(0x50C4)
#define CLK_SWCR        _SFR_(0x50C5)

/* PORT A */
#define PA_BASE_ADDRESS         0x5000
#define PA_ODR                  _SFR_(PA_BASE_ADDRESS + 0x00)
#define PA_IDR                  _SFR_(PA_BASE_ADDRESS + 0x01)
#define PA_DDR                  _SFR_(PA_BASE_ADDRESS + 0x02)
#define PA_CR1                  _SFR_(PA_BASE_ADDRESS + 0x03)
#define PA_CR2                  _SFR_(PA_BASE_ADDRESS + 0x04)

/* PORT B */
#define PB_BASE_ADDRESS         0x5005
#define PB_ODR                  _SFR_(PB_BASE_ADDRESS + 0x00)
#define PB_IDR                  _SFR_(PB_BASE_ADDRESS + 0x01)
#define PB_DDR                  _SFR_(PB_BASE_ADDRESS + 0x02)
#define PB_CR1                  _SFR_(PB_BASE_ADDRESS + 0x03)
#define PB_CR2                  _SFR_(PB_BASE_ADDRESS + 0x04)

/* PORT C */
#define PC_BASE_ADDRESS         0x500A
#define PC_ODR                  _SFR_(PC_BASE_ADDRESS + 0x00)
#define PC_IDR                  _SFR_(PC_BASE_ADDRESS + 0x01)
#define PC_DDR                  _SFR_(PC_BASE_ADDRESS + 0x02)
#define PC_CR1                  _SFR_(PC_BASE_ADDRESS + 0x03)
#define PC_CR2                  _SFR_(PC_BASE_ADDRESS + 0x04)

/* PORT D */
#define PD_BASE_ADDRESS         0x500F
#define PD_ODR                  _SFR_(PD_BASE_ADDRESS + 0x00)
#define PD_IDR                  _SFR_(PD_BASE_ADDRESS + 0x01)
#define PD_DDR                  _SFR_(PD_BASE_ADDRESS + 0x02)
#define PD_CR1                  _SFR_(PD_BASE_ADDRESS + 0x03)
#define PD_CR2                  _SFR_(PD_BASE_ADDRESS + 0x04)

/* PORT E */
#define PE_BASE_ADDRESS         0x5014
#define PE_ODR                  _SFR_(PE_BASE_ADDRESS + 0x00)
#define PE_IDR                  _SFR_(PE_BASE_ADDRESS + 0x01)
#define PE_DDR                  _SFR_(PE_BASE_ADDRESS + 0x02)
#define PE_CR1                  _SFR_(PE_BASE_ADDRESS + 0x03)
#define PE_CR2                  _SFR_(PE_BASE_ADDRESS + 0x04)

/* PORT F */
#define PF_BASE_ADDRESS         0x5019
#define PF_ODR                  _SFR_(PF_BASE_ADDRESS + 0x00)
#define PF_IDR                  _SFR_(PF_BASE_ADDRESS + 0x01)
#define PF_DDR                  _SFR_(PF_BASE_ADDRESS + 0x02)
#define PF_CR1                  _SFR_(PF_BASE_ADDRESS + 0x03)
#define PF_CR2                  _SFR_(PF_BASE_ADDRESS + 0x04)

/* PORT G */
#define PG_BASE_ADDRESS         0x501E
#define PG_ODR                  _SFR_(PG_BASE_ADDRESS + 0x00)
#define PG_IDR                  _SFR_(PG_BASE_ADDRESS + 0x01)
#define PG_DDR                  _SFR_(PG_BASE_ADDRESS + 0x02)
#define PG_CR1                  _SFR_(PG_BASE_ADDRESS + 0x03)
#define PG_CR2                  _SFR_(PG_BASE_ADDRESS + 0x04)

/* PORT H */
#define PH_BASE_ADDRESS         0x5023
#define PH_ODR                  _SFR_(PH_BASE_ADDRESS + 0x00)
#define PH_IDR                  _SFR_(PH_BASE_ADDRESS + 0x01)
#define PH_DDR                  _SFR_(PH_BASE_ADDRESS + 0x02)
#define PH_CR1                  _SFR_(PH_BASE_ADDRESS + 0x03)
#define PH_CR2                  _SFR_(PH_BASE_ADDRESS + 0x04)

/* PORT I */
#define PI_BASE_ADDRESS         0x5028
#define PI_ODR                  _SFR_(PI_BASE_ADDRESS + 0x00)
#define PI_IDR                  _SFR_(PI_BASE_ADDRESS + 0x01)
#define PI_DDR                  _SFR_(PI_BASE_ADDRESS + 0x02)
#define PI_CR1                  _SFR_(PI_BASE_ADDRESS + 0x03)
#define PI_CR2                  _SFR_(PI_BASE_ADDRESS + 0x04)

/* Flash */
#define FLASH_BASE_ADDRESS      0x505A
#define FLASH_CR1               _SFR_(FLASH_BASE_ADDRESS + 0x00)
#define FLASH_CR1_HALT          _BV(3)
#define FLASH_CR1_AHALT         _BV(2)
#define FLASH_CR1_IE            _BV(1)
#define FLASH_CR1_FIX           _BV(0)
#define FLASH_CR2               _SFR_(FLASH_BASE_ADDRESS + 0x01)
#define FLASH_CR2_OPT           _BV(7)
#define FLASH_CR2_WPRG          _BV(6)
#define FLASH_CR2_ERASE         _BV(5)
#define FLASH_CR2_FPRG          _BV(4)
#define FLASH_CR2_PRG           _BV(0)
#define FLASH_NCR2              _SFR_(FLASH_BASE_ADDRESS + 0x02)
#define FLASH_NCR2_NOPT         _BV(7)
#define FLASH_NCR2_NWPRG        _BV(6)
#define FLASH_NCR2_NERASE       _BV(5)
#define FLASH_NCR2_NFPRG        _BV(4)
#define FLASH_NCR2_NPRG         _BV(0)
#define FLASH_FPR               _SFR_(FLASH_BASE_ADDRESS + 0x03)
#define FLASH_NFPR              _SFR_(FLASH_BASE_ADDRESS + 0x04)
#define FLASH_IAPSR             _SFR_(FLASH_BASE_ADDRESS + 0x05)
#define FLASH_IAPSR_DUL         _BV(3)
#define FLASH_IAPSR_EOP         _BV(2)
#define FLASH_IAPSR_PUL         _BV(1)
#define FLASH_PUKR              _SFR_(FLASH_BASE_ADDRESS + 0x08)
#define FLASH_PUKR_KEY1         0x56
#define FLASH_PUKR_KEY2         0xAE
#define FLASH_DUKR              _SFR_(FLASH_BASE_ADDRESS + 0x0A)
#define FLASH_DUKR_KEY1         FLASH_PUKR_KEY2
#define FLASH_DUKR_KEY2         FLASH_PUKR_KEY1

/* EXTI */
#define EXTI_BASE_ADDRESS       0x50A0
#define EXTI_CR1                _SFR_(EXTI_BASE_ADDRESS + 0x00)
#define EXTI_CR2                _SFR_(EXTI_BASE_ADDRESS + 0x01)

#define EXTI_CR1_PAIS_EDGES  0b00000011
#define EXTI_CR1_PBIS_EDGES  0b00001100
#define EXTI_CR1_PCIS_EDGES  0b00110000
#define EXTI_CR1_PDIS_EDGES  0b11000000
#define EXTI_CR2_PEIS_EDGES  0b00000011
#define EXTI_CR2_TLIS_RISING 0b00000100

/* UART */
#define UART1_SR _SFR_(0x5230)
#define UART1_DR _SFR_(0x5231)
#define UART1_BRR1 _SFR_(0x5232)
#define UART1_BRR2 _SFR_(0x5233)
#define UART1_CR1 _SFR_(0x5234)
#define UART1_CR2 _SFR_(0x5235)
#define UART1_CR3 _SFR_(0x5236)
#define UART1_CR4 _SFR_(0x5237)
#define UART1_CR5 _SFR_(0x5238)
#define UART1_GTR _SFR_(0x5239)
#define UART1_PSCR _SFR_(0x523A)

#define UART_SR_TXE _BV(7)
#define UART_SR_TC _BV(6)
#define UART_SR_RXNE _BV(5)
#define UART_SR_IDLE _BV(4)
#define UART_SR_OR _BV(3)
#define UART_SR_NF _BV(2)
#define UART_SR_FE _BV(1)
#define UART_SR_PE _BV(0)

#define UART_CR1_R8 _BV(7)
#define UART_CR1_T8 _BV(6)
#define UART_CR1_UARTD _BV(5)
#define UART_CR1_M _BV(4)
#define UART_CR1_WAKE _BV(3)
#define UART_CR1_PCEN _BV(2)
#define UART_CR1_PS _BV(1)
#define UART_CR1_PIEN _BV(0)

#define UART_CR2_TIEN _BV(7)
#define UART_CR2_TCIEN _BV(6)
#define UART_CR2_RIEN _BV(5)
#define UART_CR2_ILIEN _BV(4)
#define UART_CR2_TEN _BV(3)
#define UART_CR2_REN _BV(2)
#define UART_CR2_RWU _BV(1)
#define UART_CR2_SBK _BV(0)

#define UART_CR3_LINEN _BV(6)
#define UART_CR3_STOP2 _BV(5)
#define UART_CR3_STOP1 _BV(4)
#define UART_CR3_CLKEN _BV(3)
#define UART_CR3_CPOL _BV(2)
#define UART_CR3_CPHA _BV(1)
#define UART_CR3_LBCL _BV(0)

/* Timers */
#define TIM1_CR1 _SFR_(0x5250)
#define TIM1_CR2 _SFR_(0x5251)
#define TIM1_SMCR _SFR_(0x5252)
#define TIM1_ETR _SFR_(0x5253)
#define TIM1_IER _SFR_(0x5254)
#define TIM1_SR1 _SFR_(0x5255)
#define TIM1_SR2 _SFR_(0x5256)
#define TIM1_EGR _SFR_(0x5257)
#define TIM1_CCMR1 _SFR_(0x5258)
#define TIM1_CCMR2 _SFR_(0x5259)
#define TIM1_CCMR3 _SFR_(0x525A)
#define TIM1_CCMR4 _SFR_(0x525B)
#define TIM1_CCER1 _SFR_(0x525C)
#define TIM1_CCER2 _SFR_(0x525D)
#define TIM1_CNTRH _SFR_(0x525E)
#define TIM1_CNTRL _SFR_(0x525F)
#define TIM1_PSCRH _SFR_(0x5260)
#define TIM1_PSCRL _SFR_(0x5261)
#define TIM1_ARRH _SFR_(0x5262)
#define TIM1_ARRL _SFR_(0x5263)
#define TIM1_RCR _SFR_(0x5264)
#define TIM1_CCR1H _SFR_(0x5265)
#define TIM1_CCR1L _SFR_(0x5266)
#define TIM1_CCR2H _SFR_(0x5267)
#define TIM1_CCR2L _SFR_(0x5268)
#define TIM1_CCR3H _SFR_(0x5269)
#define TIM1_CCR3L _SFR_(0x526A)
#define TIM1_CCR4H _SFR_(0x526B)
#define TIM1_CCR4L _SFR_(0x526C)
#define TIM1_BKR _SFR_(0x526D)
#define TIM1_DTR _SFR_(0x526E)
#define TIM1_OISR _SFR_(0x526F)

/* Note these are for STM8S103 and STM8S003
   STM8S105,104/207/208 are different */
#define TIM2_CR1 _SFR_(0x5300)
#define TIM2_CR2 _SFR_(0x5301)
#define TIM2_SMCR _SFR_(0x5302)
#define TIM2_IER _SFR_(0x5303)
#define TIM2_SR1 _SFR_(0x5304)
#define TIM2_SR2 _SFR_(0x5305)
#define TIM2_EGR _SFR_(0x5306)
#define TIM2_CCMR1 _SFR_(0x5307)
#define TIM2_CCMR2 _SFR_(0x5308)
#define TIM2_CCMR3 _SFR_(0x5309)
#define TIM2_CCER1 _SFR_(0x530A)
#define TIM2_CCER2 _SFR_(0x530B)
#define TIM2_CNTRH _SFR_(0x530C)
#define TIM2_CNTRL _SFR_(0x530D)
#define TIM2_PSCR _SFR_(0x530E)
#define TIM2_ARRH _SFR_(0x530F)
#define TIM2_ARRL _SFR_(0x5310)
#define TIM2_CCR1H _SFR_(0x5311)
#define TIM2_CCR1L _SFR_(0x5312)
#define TIM2_CCR2H _SFR_(0x5313)
#define TIM2_CCR2L _SFR_(0x5314)
#define TIM2_CCR3H _SFR_(0x5315)
#define TIM2_CCR3L _SFR_(0x5316)

/* Note these are for STM8S103 and STM8S003
   STM8S105,104/207/208 are different */
#define TIM4_CR1 _SFR_(0x5340)
#define TIM4_CR2 _SFR_(0x5341)
#define TIM4_SMCR _SFR_(0x5342)
#define TIM4_IER _SFR_(0x5343)
#define TIM4_SR _SFR_(0x5344)
#define TIM4_EGR _SFR_(0x5345)
#define TIM4_CNTR _SFR_(0x5346)
#define TIM4_PSCR _SFR_(0x5347)
#define TIM4_ARR _SFR_(0x5348)

#define TIM_IER_BIE _BV(7)
#define TIM_IER_TIE _BV(6)
#define TIM_IER_COMIE _BV(5)
#define TIM_IER_CC4IE _BV(4)
#define TIM_IER_CC3IE _BV(3)
#define TIM_IER_CC2IE _BV(2)
#define TIM_IER_CC1IE _BV(1)
#define TIM_IER_UIE _BV(0)

#define TIM_CR1_APRE _BV(7)
#define TIM_CR1_CMSH _BV(6)
#define TIM_CR1_CMSL _BV(5)
#define TIM_CR1_DIR _BV(4)
#define TIM_CR1_OPM _BV(3)
#define TIM_CR1_URS _BV(2)
#define TIM_CR1_UDIS _BV(1)
#define TIM_CR1_CEN _BV(0)

#define TIM_SR1_BIF _BV(7)
#define TIM_SR1_TIF _BV(6)
#define TIM_SR1_COMIF _BV(5)
#define TIM_SR1_CC4IF _BV(4)
#define TIM_SR1_CC3IF _BV(3)
#define TIM_SR1_CC2IF _BV(2)
#define TIM_SR1_CC1IF _BV(1)
#define TIM_SR1_UIF _BV(0)

/* SPI */
#define SPI_CR1 _SFR_(0x5200)
#define SPI_CR2 _SFR_(0x5201)
#define SPI_ICR _SFR_(0x5202)
#define SPI_SR _SFR_(0x5203)
#define SPI_DR _SFR_(0x5204)
#define SPI_CRCPR _SFR_(0x5205)
#define SPI_RXCRCR _SFR_(0x5206)
#define SPI_TXCRCR _SFR_(0x5207)

#define SPI_CR1_LSBFIRST _BV(7)
#define SPI_CR1_SPE _BV(6)
#define SPI_CR1_BR(br) ((br) << 3)
#define SPI_CR1_MSTR _BV(2)
#define SPI_CR1_CPOL _BV(1)
#define SPI_CR1_CPHA _BV(0)

#define SPI_CR2_BDM _BV(7)
#define SPI_CR2_BDOE _BV(6)
#define SPI_CR2_CRCEN _BV(5)
#define SPI_CR2_CRCNEXT _BV(4)
#define SPI_CR2_RXONLY _BV(2)
#define SPI_CR2_SSM _BV(1)
#define SPI_CR2_SSI _BV(0)

#define SPI_ICR_TXIE _BV(7)
#define SPI_ICR_RXIE _BV(6)
#define SPI_ICR_ERRIE _BV(5)
#define SPI_ICR_WKIE _BV(4)

#define SPI_SR_BSY _BV(7)
#define SPI_SR_OVR _BV(6)
#define SPI_SR_MODF _BV(5)
#define SPI_SR_CRCERR _BV(4)
#define SPI_SR_WKUP _BV(3)
#define SPI_SR_TXE _BV(1)
#define SPI_SR_RxNE _BV(0)

/* I2C */
#define I2C_CR1 _SFR_(0x5210)
#define I2C_CR2 _SFR_(0x5211)
#define I2C_FREQR _SFR_(0x5212)
#define I2C_OARL _SFR_(0x5213)
#define I2C_OARH _SFR_(0x5214)
#define I2C_DR _SFR_(0x5216)
#define I2C_SR1 _SFR_(0x5217)
#define I2C_SR2 _SFR_(0x5218)
#define I2C_SR3 _SFR_(0x5219)
#define I2C_ITR _SFR_(0x521A)
#define I2C_CCRL _SFR_(0x521B)
#define I2C_CCRH _SFR_(0x521C)
#define I2C_TRISER _SFR_(0x521D)
#define I2C_PECR _SFR_(0x521E)

/* ADC */
#define ADC_DBxR _SFR_(0x53E0)
#define ADC_CSR _SFR_(0x5400)
#define ADC_CR1 _SFR_(0x5401)
#define ADC_CR2 _SFR_(0x5402)
#define ADC_CR3 _SFR_(0x5403)
#define ADC_DRH _SFR_(0x5404)
#define ADC_DRL _SFR_(0x5405)
#define ADC_TDRH _SFR_(0x5406)
#define ADC_TDRL _SFR_(0x5407)
#define ADC_HTRH _SFR_(0x5408)
#define ADC_HTRL _SFR_(0x5409)
#define ADC_LTRH _SFR_(0x540A)
#define ADC_LTRL _SFR_(0x540B)
#define ADC_AWSRH _SFR_(0x540C)
#define ADC_AWSRL _SFR_(0x540D)
#define ADC_AWCRH _SFR_(0x540E)
#define ADC_AWCRL _SFR_(0x540F)

#define ADC_CSR_EOC _BV(7)
#define ADC_CSR_AWD _BV(6)
#define ADC_CSR_EOCIE _BV(5)
#define ADC_CSR_AWDIE _BV(4)

#define ADC_CR1_CONT _BV(1)
#define ADC_CR1_ADON _BV(0)

#define ADC_CR2_EXTTRIG _BV(6)
#define ADC_CR2_EXTSEL _BV(4)
#define ADC_CR2_ALIGN _BV(3)
#define ADC_CR2_SCAN _BV(1)

/* Interrupt commands */
#define enable_interrupts()   __asm__("rim")
#define disable_interrupts()  __asm__("sim")
#define nop()                 __asm__("nop")  /* No Operation */
#define trap()                __asm__("trap") /* Trap (soft IT) */
#define wfi()                 __asm__("wfi")  /* Wait For Interrupt */
#define halt()                __asm__("halt") /* Halt */

/* Interrupts */
#define TLI_ISR                 0
#define AWU_ISR                 1
#define CLK_ISR                 2
#define EXTI0_ISR               3
#define EXTI1_ISR               4
#define EXTI2_ISR               5
#define EXTI3_ISR               6
#define EXTI4_ISR               7
#define CAN_RX_ISR              8   /* dual use, device dependent */
#define EXTI5_ISR               8   /* dual use, device dependent */
#define CAN_TX_ISR              9
#define SPI_ISR                 10
#define TIM1_OVF_ISR            11
#define TIM1_CC_ISR             12
#define TIM2_OVF_ISR            13   /* dual use, device dependent */
#define TIM5_OVF_ISR            13   /* dual use, device dependent */
#define TIM2_CC_ISR             14   /* dual use, device dependent */
#define TIM5_CC_ISR             14   /* dual use, device dependent */
#define TIM3_OVF_ISR            15
#define TIM3_CC_ISR             16
#define UART1_TXC_ISR           17
#define UART1_RXC_ISR           18
#define I2C_ISR                 19
#define UART2_3_4_TXC_ISR       20
#define UART2_3_4_RXC_ISR       21
#define ADC1_ISR                22   /* dual use, device dependent */
#define ADC2_ISR                22   /* dual use, device dependent */
#define TIM4_ISR                23   /* dual use, device dependent */
#define TIM6_ISR                23   /* dual use, device dependent */
#define FLASH_ISR               24
