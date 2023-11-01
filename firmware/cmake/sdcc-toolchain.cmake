# Change compiler and target
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_C_COMPILER sdcc)
SET(CMAKE_ASM_COMPILER sdasstm8)

set(CMAKE_C_FLAGS_INIT "-mstm8 --std-sdcc2x")

# Find paths
find_program (SDCC NAMES sdcc)
get_filename_component(SDCC_BIN_DIR ${SDCC} DIRECTORY)
get_filename_component(SDCC_PATH_DIR ${SDCC_BIN_DIR} DIRECTORY)

# here is the target environment is located
#SET(CMAKE_FIND_ROOT_PATH  ${SDCC_PATH_DIR}/usr/share/sdcc)

# Adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search
# programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_ASM_COMPILE_OBJECT "${CMAKE_ASM_COMPILER} -o <OBJECT> <SOURCE>")
