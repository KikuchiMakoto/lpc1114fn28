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
INCLUDES = -I$(LPCOPEN)/inc

# ==== Build type (Debug or Release) ====
BUILD ?= release  # デフォルトは release

ifeq ($(BUILD),debug)
    CFLAGS_EXTRA = -O0 -g -ggdb -DDEBUG
	LDFLAGS_EXTRA = 
    OUTDIR = out/debug
else
    # 高度最適化 + サイズ重視 + CPU 特化
    CFLAGS_EXTRA = -Os -flto \
				   -Wl,--gc-sections \
				   -fmerge-constants -fipa-pta \
                   -fomit-frame-pointer -fno-common \
				   -ffunction-sections -fdata-sections \
				   -fstrict-aliasing -fmerge-all-constants
    LDFLAGS_EXTRA = -flto
    OUTDIR = out/release
endif

# ==== Source files ====
C_SOURCES   = $(SRCDIR)/cr_startup_lpc11xx.c \
              $(wildcard $(LPCOPEN)/src/*.c)
CPP_SOURCES = $(SRCDIR)/main.cpp

# ==== Object files ====
OBJS = $(addprefix $(OBJDIR)/,$(notdir $(C_SOURCES:.c=.o))) \
       $(addprefix $(OBJDIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))

# ==== Compiler/Linker flags ====
CFLAGS  = -DCORE_M0 -DUSE_OLD_STYLE_DATA_BSS_INIT \
		  -mcpu=cortex-m0 -mthumb -Wall -Wextra \
		  $(CFLAGS_EXTRA) $(INCLUDES)
CXXFLAGS= $(CFLAGS) -fno-exceptions -fno-rtti
LDFLAGS = -T$(SRCDIR)/lpc1114.ld \
          -Wl,-Map=$(OUTDIR)/$(TARGET).map \
		  -nostdlib \
          -specs=nosys.specs -specs=nano.specs \
          -lc -lgcc $(LDFLAGS_EXTRA)

# ==== Build rules ====
all: $(OBJDIR) $(OUTDIR) \
     $(OUTDIR)/$(TARGET).elf \
     $(OUTDIR)/$(TARGET).bin \
     $(OUTDIR)/$(TARGET).hex \
     $(OUTDIR)/$(TARGET).lst

# Create directories
$(OBJDIR):
	if not exist "$(OBJDIR)" mkdir "$(OBJDIR)"

$(OUTDIR):
	if not exist "$(OUTDIR)" mkdir "$(OUTDIR)"

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
	if exist "$(OBJDIR)" rd /S /Q "$(OBJDIR)"
	if exist "$(OUTDIR)" rd /S /Q "$(OUTDIR)"
