#pragma once

#define HNSEC_PER_USEC      10
#define HNSEC_PER_MSEC      10000
#define HNSEC_PER_SEC       10000000
#define HNSEC_PER_MINUTE    (HNSEC_PER_SEC * 60)
#define NSEC_PER_HNSEC      100

void motor_timer_init(void);
uint64_t motor_timer_hnsec(void);
void motor_timer_set_relative(int64_t delay_hnsec);
int64_t motor_timer_set_absolute(uint64_t timestamp_hnsec);
void motor_timer_cancel(void);
void motor_timer_hndelay(int hnsecs);

extern void motor_timer_callback(uint64_t timestamp_hnsec);
