#include "include.h"

// master config struct with data independent from profiles
master_t masterConfig;

static void resetConfigs(void)
{
    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));

    // *****************************************************************************
    motor_pwm_Config()->mot_pwm_hz = 24000;
    motor_pwm_Config()->mot_pwm_dt_ns = 600;
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
            return;
    }
    resetEEPROM();
}

void resetEEPROM(void)
{
    memset(&masterConfig, 0, sizeof(master_t));
    masterConfig.version = EEPROM_CONF_VERSION;

    resetConfigs();

    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
}
