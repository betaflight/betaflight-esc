#
# F0 Make file include
#
CMSIS_DIR       := $(LIB_DIR)/$(TARGET_MCU)/Drivers/CMSIS

DRIVER_DIR       = $(LIB_DIR)/$(TARGET_MCU)/Drivers/STM32F0xx_HAL_Driver
DRIVER_SRC       = $(notdir $(wildcard $(DRIVER_DIR)/Src/*.c))
EXCLUDES         = \
				   stm32f0xx_rtc.c \
                   stm32f0xx_hal_timebase_rtc_wakeup_template.c \
                   stm32f0xx_hal_timebase_rtc_alarm_template.c \
                   stm32f0xx_hal_timebase_tim_template.c

STARTUP_SRC      = startup_$(STM_CHIP)_gcc.s
DRIVER_SRC      := $(filter-out ${EXCLUDES}, $(DRIVER_SRC))

# Search path and source files for the CMSIS sources
CMSIS_SRC        = $(notdir $(wildcard $(LIB_DIR)/Device/*.c))
VPATH           := $(VPATH):$(LIB_DIR):$(LIB_DIR)/Device:$(DRIVER_DIR)/Src

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(DRIVER_DIR)/Inc \
                   $(DRIVER_DIR)/Inc/Legacy \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/ST/STM32F0xx/Include

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f0xx_$(FLASH_SIZE)k.ld
ARCH_FLAGS      = -mthumb -mcpu=cortex-m0

DEVICE_FLAGS   += -DSTM32F0XX -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER

MCU_SRC        := target/system_stm32f0xx.c

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Os
OPTIMISE_SPEED      :=
OPTIMISE_SIZE       :=

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
endif
