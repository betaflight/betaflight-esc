#include "include.h"

// master config struct with data independent from profiles
master_t masterConfig;

static void resetConfigs(void)
{
    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));

    // *****************************************************************************
    motor_pwm_Config()->mot_pwm_hz = 24000;// 默认为24Khz进行控制MOS切换
    motor_pwm_Config()->mot_pwm_dt_ns = 600;// 根据FD6288选型,死区时间为600ns

    // *****************************************************************************
    rpmctl_Config()->rpmctl_p = 0.0001;// RPM loop PID proportional gain(0.0 - 1.0)
    rpmctl_Config()->rpmctl_d = 0.0;// RPM loop PID integral gain(0.0 - 1.0)
    rpmctl_Config()->rpmctl_i = 0.001;// RPM loop PID derivative gain(0.0 - 10.0)

    // *****************************************************************************
    motor_signal_Config()->pwm_min_usec = 1000;// (800 - 1200)
    motor_signal_Config()->pwm_max_usec = 2000;// (1800 - 2200)

    // *****************************************************************************
    // Timing advance settings
    motor_rtctl_Config()->mot_tim_adv_min = 5;// (0 - 20) electrical degree
    motor_rtctl_Config()->mot_tim_adv_max = 15;// (6 - 500) electrical degree
    motor_rtctl_Config()->mot_tim_cp_max = 300;// (6 - 500) microsecond
    motor_rtctl_Config()->mot_tim_cp_min = 600;// (6 - 500) microsecond
    // Most important parameters
    motor_rtctl_Config()->mot_blank_usec = 40;// (10 - 300) microsecond
    motor_rtctl_Config()->mot_zc_fails_max = 20;// (6 - 300) dimensionless
    motor_rtctl_Config()->mot_comm_per_max = 4000;// (1000 - 10000) microsecond
    // Spinup settings
    motor_rtctl_Config()->mot_spup_st_cp = 100000;// (10000 - 300000) microsecond
    motor_rtctl_Config()->mot_spup_to_ms = 5000;// (100 - 9000) millisecond (sic!)
    motor_rtctl_Config()->mot_spup_blnk_pm = 100;// (1 - 300) permill

    // *****************************************************************************
    motor_Config()->mot_v_min = 2.5;// (0.5 - 10.0)
    motor_Config()->mot_v_spinup = 0.5;// (0.01 - 10.0)
    motor_Config()->mot_spup_vramp_t = 3.0;// (0.0 - 10.0)
    motor_Config()->mot_dc_accel = 0.09;// (0.001 - 0.5)
    motor_Config()->mot_dc_slope = 5.0;// (1.0 - 20.0)

    motor_Config()->mot_num_poles = 14;// (2 - 100)
    motor_Config()->ctl_dir = 0;// (0 - 1)

    motor_Config()->mot_rpm_min = 1000;// (50 - 5000)

    motor_Config()->mot_i_max = 20.0;// (1.0 - 60.0)
    motor_Config()->mot_i_max_p = 0.2;// (0.01 - 2.0)

    motor_Config()->mot_lpf_freq = 20.0;// (1.0 - 200.0)
    motor_Config()->mot_stop_thres = 7;// (1 - 100)
}

/**
 * 确保是EEPROM中存放有效数据
 */
void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid())
        return;

    // 重置参数
    resetEEPROM();
}

/**
 * 重置EEPROM中的数据
 */
void resetEEPROM(void)
{
    // 清空所有的配置信息
    memset(&masterConfig, 0, sizeof(master_t));
    masterConfig.version = EEPROM_CONF_VERSION;

    // 重新写入配置
    resetConfigs();

    // 写入Flash中
    writeEEPROM();
}

/**
 * 保存EEPROM中的数据
 */
void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
}
