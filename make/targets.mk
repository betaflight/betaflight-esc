OFFICIAL_TARGETS  = FISHDRONE

VALID_TARGETS   = $(dir $(wildcard $(ROOT)/src/main/target/*/target.mk))
VALID_TARGETS  := $(subst /,, $(subst ./src/main/target/,, $(VALID_TARGETS)))
VALID_TARGETS  := $(VALID_TARGETS) $(ALT_TARGETS)
VALID_TARGETS  := $(sort $(VALID_TARGETS))

SUPPORTED_TARGETS = $(VALID_TARGETS)

-include $(ROOT)/src/main/target/$(TARGET)/target.mk

ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS). Have you prepared a valid target.mk?)
endif

ifeq ($(filter $(TARGET),$(F0_TARGETS) $(F3_TARGETS)),)
$(error Target '$(TARGET)' has not specified a valid STM group, must be one of F0, or F3. Have you prepared a valid target.mk?)
endif

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
TARGET_MCU := STM32F3

else ifeq ($(TARGET),$(filter $(TARGET), $(F0_TARGETS)))
TARGET_MCU := STM32F0

else
$(error Unknown target MCU specified.)
endif

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)
