# ==== Toolchain ====
CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
AS      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# ==== Target ====
TARGET = main

# ==== Source/Include paths ====
SRCDIR  = src
LPCOPEN = lpc_chip_11cxx_lib
OBJDIR  = obj
OUTDIR  = out
INCLUDES = -I$(LPCOPEN)/inc

# ==== Source files ====
C_SOURCES   = $(SRCDIR)/cr_startup_lpc11xx.c \
              $(wildcard $(LPCOPEN)/src/*.c)
CPP_SOURCES = $(SRCDIR)/main.cpp

# ==== Object files in obj/ ====
OBJS = $(addprefix $(OBJDIR)/,$(notdir $(C_SOURCES:.c=.o))) \
       $(addprefix $(OBJDIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))

# ==== Compiler/Linker flags ====
CFLAGS  = -DCORE_M0 -DUSE_OLD_STYLE_DATA_BSS_INIT \
          -mcpu=cortex-m0 -mthumb -Wall -Wextra -O2 $(INCLUDES)
CXXFLAGS= $(CFLAGS) -fno-exceptions -fno-rtti
LDFLAGS = -T$(SRCDIR)/lpc1114.ld \
          -Wl,--gc-sections \
          -Wl,-Map=$(OUTDIR)/$(TARGET).map

# ==== Build rules ====
all: $(OBJDIR) $(OUTDIR) \
     $(OUTDIR)/$(TARGET).elf \
     $(OUTDIR)/$(TARGET).bin \
     $(OUTDIR)/$(TARGET).hex \
     $(OUTDIR)/$(TARGET).lst

$(OBJDIR):
	@if not exist $(OBJDIR) mkdir $(OBJDIR)

$(OUTDIR):
	@if not exist $(OUTDIR) mkdir $(OUTDIR)

# ELF
$(OUTDIR)/$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) $@

# BIN
$(OUTDIR)/$(TARGET).bin: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# HEX
$(OUTDIR)/$(TARGET).hex: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# LST
$(OUTDIR)/$(TARGET).lst: $(OUTDIR)/$(TARGET).elf
	$(OBJDUMP) -d -S $< > $@

# Object rules
$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(LPCOPEN)/src/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: all clean

clean:
	@if exist $(OBJDIR) rd /s /q $(OBJDIR)
	@if exist $(OUTDIR) rd /s /q $(OUTDIR)
