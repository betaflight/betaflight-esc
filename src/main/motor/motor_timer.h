#pragma once

#define HNSEC_PER_USEC      10 // 1微秒 = 10亚微秒
#define HNSEC_PER_MSEC      10000// 1毫秒 = 10000亚微秒
#define HNSEC_PER_SEC       10000000// 1秒 = 10000000亚微秒
#define HNSEC_PER_MINUTE    (HNSEC_PER_SEC * 60)// 1分钟 = 10000000亚微秒 * 60
#define NSEC_PER_HNSEC      100

/**
 * 初始化电机定时器14和定时器17
 */
void motor_timer_init(void);

/**
 * 返回当前时间戳(亚微秒级) Returns the current timestamp in hectonanoseconds (10^-7).
 */
uint64_t motor_timer_hnsec(void);

/**
 * 设置相对的延时时间(亚纳秒),延时结束后回调对应函数,返回增量时间
 */
void motor_timer_set_relative(int64_t delay_hnsec);

/**
 * 设置绝对的延时时间(亚纳秒),延时结束后回调对应函数,返回增量时间
 */
int64_t motor_timer_set_absolute(uint64_t timestamp_hnsec);

/**
 * 取消定时器的定时任务
 */
void motor_timer_cancel(void);

/**
 * 精确延时函数
 */
void motor_timer_hndelay(int hnsecs);

/**
 * 定时器电机回调函数
 */
extern void motor_timer_callback(uint64_t timestamp_hnsec);
