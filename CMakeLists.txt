cmake_minimum_required(VERSION 3.15)

# ==== Generator default (Ninja優先) ====
if(NOT CMAKE_GENERATOR)
    find_program(NINJA_EXECUTABLE ninja)
    if(NINJA_EXECUTABLE)
        set(CMAKE_GENERATOR "Ninja" CACHE INTERNAL "" FORCE)
        message(STATUS "Default generator set to Ninja")
    else()
        set(CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)
        message(STATUS "Ninja not found, using Unix Makefiles")
    endif()
endif()

# ==== Toolchain Setup (project()の前に記述) ====
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ARM toolchain
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# コンパイラテストをスキップ（クロスコンパイル環境）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ==== Project ====
project(LPC1114_Project C CXX ASM)

# ==== Binary tools ====
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_OBJDUMP arm-none-eabi-objdump REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size REQUIRED)

# ==== Target ====
set(TARGET_NAME main)

# ==== Directories ====
set(SRCDIR ${CMAKE_SOURCE_DIR}/src)
set(LPCOPEN ${CMAKE_SOURCE_DIR}/lpc_chip_11cxx_lib)

# ==== Source files ====
file(GLOB C_SOURCES 
    ${SRCDIR}/cr_startup_lpc11xx.c
    ${LPCOPEN}/src/*.c
)

set(CPP_SOURCES
    ${SRCDIR}/main.cpp
)

# ==== Compiler flags ====
add_compile_definitions(
    CORE_M0
    USE_OLD_STYLE_DATA_BSS_INIT
)

add_compile_options(
    -mcpu=cortex-m0
    -mthumb
    -Wall
    -Wextra
    -O2
)

# C++専用フラグ
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti")

# ==== Linker flags ====
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \
    -T${SRCDIR}/lpc1114.ld \
    -Wl,--gc-sections \
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map"
)

# ==== Include directories ====
include_directories(${LPCOPEN}/inc)

# ==== Executable ====
add_executable(${TARGET_NAME}.elf
    ${C_SOURCES}
    ${CPP_SOURCES}
)

# ==== Post-build: Generate BIN, HEX, LST ====
add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}.elf> ${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJDUMP} -d -S $<TARGET_FILE:${TARGET_NAME}.elf> > ${TARGET_NAME}.lst
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}.elf>
    COMMENT "Generating BIN, HEX, LST files and showing size..."
    VERBATIM
)

# ==== Display output files ====
message(STATUS "=== Build Configuration ===")
message(STATUS "  Generator: ${CMAKE_GENERATOR}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Target: ${TARGET_NAME}.elf")
message(STATUS "  Linker script: ${SRCDIR}/lpc1114.ld")
message(STATUS "  Output directory: ${CMAKE_BINARY_DIR}")