#pragma once

#define motor_pwm_Config(x) (&masterConfig.motor_pwm_Config)

typedef struct master_s {
    uint8_t version;
    uint16_t size;
    
    // magic number, should be 0xBE
    uint8_t magic_be;                       

    motor_pwm_Config_t motor_pwm_Config;

    // magic number, should be 0xEF
    uint8_t magic_ef;
    uint8_t chk;
} master_t;

extern master_t masterConfig;
