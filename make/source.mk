COMMON_SRC = \
                    $(TARGET_DIR_SRC) \
                    main.c

FW_SRC = \

COMMON_DEVICE_SRC = \
                    $(CMSIS_SRC) \
                    $(DEVICE_STDPERIPH_SRC)

COMMON_SRC          := $(COMMON_SRC) $(FW_SRC) $(COMMON_DEVICE_SRC)


SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

# check if target.mk supplied
SRC                 := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) $(VARIANT_SRC) $(COMMON_SRC)

#excludes
SRC                 := $(filter-out ${MCU_EXCLUDES}, $(SRC))

# Search path and source files for the ST stdperiph library
VPATH               := $(VPATH):$(STDPERIPH_DIR)/src
