#pragma once

#pragma diag_suppress 188

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define MOTOR_NUM_PHASES        3
#define MOTOR_NUM_COMMUTATION_STEPS      6

#define MIN(a, b)                  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                  (((a) > (b)) ? (a) : (b))

/************** 支持层 *******************/
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

/********** STM32硬件平台层 *********/
#include "stm32f0xx.h"

/************** 通用层 *******************/
#include "irq.h"
#include "multi_timer.h"

/************** 驱动层 *******************/
#include "drv_system.h"
#include "drv_uart.h"
#include "drv_led.h"
#include "drv_watchdog.h"

/************** 电机层 *******************/
#include "motor_adc.h"
#include "motor_comparator.h"
#include "motor_pwm.h"
#include "motor_signal.h"
#include "motor_telemetry.h"
#include "motor_timer.h"
#include "motor_rtctl.h"
#include "motor_rtctl_test.h"

/************** 控制层 *******************/
#include "rpmctl.h"
#include "control.h"

/************** 配置层 *******************/
#include "config.h"
#include "config_master.h"
#include "config_eeprom.h"
