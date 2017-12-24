
TARGET_DIR     = $(SRC_DIR)/target/$(TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

INCLUDE_DIRS  := $(SRC_DIR) \
                 $(SRC_DIR)/target \
                 $(TARGET_DIR) \
                 $(INCLUDE_DIRS)

VPATH         := $(VPATH):$(TARGET_DIR)

COMMON_SRC           = \
                    $(TARGET_DIR_SRC) \
                    main.c \
                    config/config.c \
                    config/config_eeprom.c \
                    control/control.c \
                    control/rpmctl.c \
                    drivers/drv_led.c \
                    drivers/drv_system.c \
                    drivers/drv_uart.c \
                    drivers/drv_watchdog.c \
                    drivers/multi_timer.c \
                    motor/motor_adc.c \
                    motor/motor_comparator.c \
                    motor/motor_pwm.c \
                    motor/motor_rtctl.c \
                    motor/motor_rtctl_test.c \
                    motor/motor_signal.c \
                    motor/motor_telemetry.c \
                    motor/motor_timer.c

VPATH               := $(VPATH):$(SRC_DIR)

SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

# check if target.mk supplied
SRC                 := $(STARTUP_DIR)/$(STARTUP_SRC) $(COMMON_SRC) $(MCU_SRC) $(CMSIS_SRC) $(DRIVER_SRC)

#excludes
SRC                 := $(filter-out ${MCU_EXCLUDES}, $(SRC))

