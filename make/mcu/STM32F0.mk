#
# F0 Make file include
#
STDPERIPH_DIR    = $(ROOT)/lib/main/STM32F0/Drivers/STM32F0xx_StdPeriph_Driver
STDPERIPH_SRC    = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES         = \
				   stm32f0xx_rtc.c

STARTUP_SRC      = startup_$(STM_CHIP)_gcc.s
STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

# Search path and source files for the CMSIS sources
VPATH           := $(VPATH):$(CMSIS_DIR)/Include:$(CMSIS_DIR)/Device
CMSIS_SRC        = $(notdir $(wildcard $(CMSIS_DIR)/Device/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f0xx_$(FLASH_SIZE)k.ld
ARCH_FLAGS      = -mthumb -mcpu=cortex-m0

DEVICE_FLAGS   += -DSTM32F0XX

MCU_COMMON_SRC = \

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      :=
OPTIMISE_SIZE       :=

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif
