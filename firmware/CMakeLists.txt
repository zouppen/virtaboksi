cmake_minimum_required(VERSION 3.25)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdcc-toolchain.cmake")

project(virtaboksi C ASM)

# Product filename
set(PRODUCT_NAME virtaboksi)

# Set STM8 device type
set(DEVICE stm8s003?3 CACHE STRING "STM8 variant")

# RC oscillator frequency. This should be one of the supported values in your MCU.
set(F_CPU 2000000 CACHE STRING "CPU clock speed")

# Default Baudrate for UART
set(BAUD 9600 CACHE STRING "Serial port baud rate")

set(CLOCK_PRESCALER 1000)

# MODBUS_SILENCE is the 14 bit long duration on the serial line,
# measured in ticks. We consider a frame to be ready after 14
# bits and after another 14 bits we can start transmitting.
# https://en.wikipedia.org/wiki/Modbus#Modbus_RTU_frame_format_(primarily_used_on_asynchronous_serial_data_lines_like_RS-485/EIA-485)
# This trick does rounding towards higher integer
math(EXPR MODBUS_SILENCE_DIV "${BAUD} * ${CLOCK_PRESCALER}" OUTPUT_FORMAT DECIMAL)
math(EXPR MODBUS_SILENCE "(14 * ${F_CPU} + ${MODBUS_SILENCE_DIV} - 1) / ${MODBUS_SILENCE_DIV}" OUTPUT_FORMAT DECIMAL)

# The programmer to use, see stm8flash -c allowed values
set(PROGRAMMER stlinkv2 CACHE STRING "Programmer type in stm8flash")

# Pass defines to compiler
add_definitions(
  -DF_CPU=${F_CPU}UL
  -DBOARD_0_5
  -DBAUD=${BAUD}
  -DMODBUS_SILENCE=${MODBUS_SILENCE}
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
