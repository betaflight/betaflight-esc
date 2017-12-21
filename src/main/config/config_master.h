#pragma once

#define rpmctl_Config(x) (&masterConfig.rpmctl_Config)
#define motor_pwm_Config(x) (&masterConfig.motor_pwm_Config)
#define motor_signal_Config(x) (&masterConfig.motor_signal_Config)
#define motor_rtctl_Config(x) (&masterConfig.motor_rtctl_Config)
#define motor_Config(x) (&masterConfig.motor_Config)

typedef struct master_s {
    uint8_t version;
    uint16_t size;
    
    // magic number, should be 0xBE
    uint8_t magic_be;                       
    
    rpmctl_Config_t rpmctl_Config;
    motor_pwm_Config_t motor_pwm_Config;
    motor_signal_Config_t motor_signal_Config;
    motor_rtctl_Config_t motor_rtctl_Config;
    motor_Config_t motor_Config;

    // magic number, should be 0xEF
    uint8_t magic_ef;
    uint8_t chk;
} master_t;

extern master_t masterConfig;
