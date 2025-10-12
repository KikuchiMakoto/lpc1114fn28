cmake_minimum_required(VERSION 3.21)

# ==== Toolchain configuration ====
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Skip compiler checks (embedded systems)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ==== Project ====
project(lpc1114_blinky C CXX ASM)

# ==== Paths ====
set(SRCDIR ${CMAKE_SOURCE_DIR}/src)
set(LPCOPEN ${CMAKE_SOURCE_DIR}/lpc_chip_11cxx_lib)
set(FREERTOS ${CMAKE_SOURCE_DIR}/freertos)

# ==== MCU specific settings ====
set(MCU_FLAGS
    -mcpu=cortex-m0
    -mthumb
)

# ==== Critical Definitions ====
add_compile_definitions(
    CORE_M0
    USE_OLD_STYLE_DATA_BSS_INIT
    __OPTIMIZE_SIZE__
)

add_compile_options(
    ${MCU_FLAGS}
    -Os
    -g0
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fmerge-all-constants
    -fno-unwind-tables
    -fno-asynchronous-unwind-tables
    -fno-math-errno
    -Wall
    -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)

# ==== Linker Flags ====
set(LINKER_SCRIPT ${SRCDIR}/lpc1114.ld)
add_link_options(
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--print-gc-sections
    -Wl,-Map=${PROJECT_BINARY_DIR}/main.map
    -Wl,--cref
    -Wl,--no-warn-mismatch
    -Wl,--print-memory-usage
    -Wl,--sort-section=alignment
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
)

# ==== Include directories ====
include_directories(${LPCOPEN}/inc)
include_directories(${FREERTOS}/inc)

# ==== Source files ====
file(GLOB C_SOURCES ${LPCOPEN}/src/*.c ${FREERTOS}/src/*.c)
set(C_SOURCES 
    ${C_SOURCES}
    ${SRCDIR}/cr_startup_lpc11xx.c
)

set(CPP_SOURCES
    ${SRCDIR}/main.cpp
)

# ==== Executable target ====
set(TARGET main)
add_executable(${TARGET}.elf
    ${C_SOURCES}
    ${CPP_SOURCES}
)

# Ensure linker script is a dependency
set_target_properties(${TARGET}.elf PROPERTIES
    LINK_DEPENDS ${LINKER_SCRIPT}
)

# ==== Post-build: Generate .bin, .hex, .lst ====
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET}.elf ${TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET}.elf ${TARGET}.hex
    COMMAND ${CMAKE_OBJDUMP} -d -S ${TARGET}.elf > ${TARGET}.lst
    COMMAND ${CMAKE_SIZE} ${TARGET}.elf
    COMMENT "Generating binary, hex, and listing files..."
)

# ==== Clean additional files ====
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/main.bin"
    "${CMAKE_BINARY_DIR}/main.hex"
    "${CMAKE_BINARY_DIR}/main.lst"
    "${CMAKE_BINARY_DIR}/main.map"
)