cmake_minimum_required(VERSION 3.25)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdcc-toolchain.cmake")

project(virtaboksi C ASM)

# Product filename
set(PRODUCT_NAME virtaboksi)

# Hardware versions
set(BOARD_VERSION 0.5 CACHE STRING "Hardware version")
set_property(CACHE BOARD_VERSION PROPERTY STRINGS 0.4 0.5)

# RC oscillator frequency. This should be one of the supported values in your MCU.
set(F_CPU_MHZ 2 CACHE STRING "CPU clock speed")
set_property(CACHE F_CPU_MHZ PROPERTY STRINGS 2 4 8 16)
math(EXPR F_CPU "${F_CPU_MHZ} * 1000000" OUTPUT_FORMAT DECIMAL)

# Default Baudrate for UART
set(BAUD 9600 CACHE STRING "Serial port default baud rate")

# Tick granularity. Too low means more interrupts.
set(TICK_USEC 1000 CACHE STRING "Tick duration in µs")

# The programmer to use, see stm8flash -c allowed values
set(PROGRAMMER stlinkv2 CACHE STRING "Programmer type in stm8flash")

# Some operational variables which aren't yet run-time configurable
set(DEBOUNCE_MS 200 CACHE STRING "Debounce duration in ms")
set(STARTUP_DEBOUNCE_MS 500 CACHE STRING "Startup debounce in ms")
set(PANIC_OFF_MS 1000 CACHE STRING "Panic off in case of a too long debouning, in ms")
set(MINIMUM_WAKEUP_MS 100 CACHE STRING "Minimum time to stay awake after a wake-up, in ms")
set(SERIAL_KEEPALIVE_MS 1000 CACHE STRING "Minimum time to stay awake after serial activity, in ms")

math(EXPR DEBOUNCE_TICKS "${DEBOUNCE_MS} * 1000 / ${TICK_USEC}" OUTPUT_FORMAT DECIMAL)
math(EXPR STARTUP_DEBOUNCE_TICKS "${STARTUP_DEBOUNCE_MS} * 1000 / ${TICK_USEC}" OUTPUT_FORMAT DECIMAL)
math(EXPR PANIC_OFF_TICKS "${PANIC_OFF_MS} * 1000 / ${TICK_USEC}" OUTPUT_FORMAT DECIMAL)
math(EXPR MINIMUM_WAKEUP_TICKS "${MINIMUM_WAKEUP_MS} * 1000 / ${TICK_USEC}" OUTPUT_FORMAT DECIMAL)
math(EXPR SERIAL_KEEPALIVE_TICKS "${SERIAL_KEEPALIVE_MS} * 1000 / ${TICK_USEC}" OUTPUT_FORMAT DECIMAL)

# Report what we are going to do
message(STATUS "Board version: ${BOARD_VERSION}")
message(STATUS "CPU frequency: ${F_CPU_MHZ} MHz")
message(STATUS "Default baud rate: ${BAUD} bps")

# Set some variables based on hardware version
if(BOARD_VERSION STREQUAL "0.4")
  set(DEVICE stm8s003?3)
  add_definitions(-DBOARD_0_4)
elseif(BOARD_VERSION STREQUAL "0.5")
  set(DEVICE stm8s003?3)
  add_definitions(-DBOARD_0_5)
else()
  message(SEND_ERROR "Unsupported board version")
endif()

# Pass defines to compiler
add_definitions(
  -DF_CPU=${F_CPU}UL
  -DBAUD=${BAUD}UL
  -DTICK_USEC=${TICK_USEC}U
  -DDEBOUNCE_TICKS=${DEBOUNCE_TICKS}U
  -DSTARTUP_DEBOUNCE_TICKS=${STARTUP_DEBOUNCE_TICKS}U
  -DPANIC_OFF_TICKS=${PANIC_OFF_TICKS}U
  -DMINIMUM_WAKEUP_TICKS=${MINIMUM_WAKEUP_TICKS}U
  -DSERIAL_KEEPALIVE_TICKS=${SERIAL_KEEPALIVE_TICKS}U
)

add_compile_options(
  --Werror
)

add_link_options(
  -lstm8
)

# STM8 library
file(GLOB_RECURSE STM8LIB_FILES "stm8-lib/[^.]*.c")
add_library(stm8-lib OBJECT ${STM8LIB_FILES})

# Our code
file(GLOB_RECURSE SRC_FILES "src/[^.]*.c")
add_executable(${PRODUCT_NAME} ${SRC_FILES} $<TARGET_OBJECTS:stm8-lib>)
target_include_directories(${PRODUCT_NAME} PRIVATE stm8-lib)

# Upload the firmware
add_custom_target(upload PROGRAMMER=${PROGRAMMER} DEVICE=${DEVICE} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/uploader $<TARGET_FILE:${PRODUCT_NAME}> DEPENDS ${PRODUCT_NAME})
