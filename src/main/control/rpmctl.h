#pragma once

typedef struct rpmctl_Config_s {
    double rpmctl_p;// RPM loop PID proportional gain
    double rpmctl_i;// RPM loop PID integral gain
    double rpmctl_d;// RPM loop PID derivative gain
} rpmctl_Config_t;

/**
 * 控制模块输入值
 */
struct rpmctl_input
{
    int limit_mask;// 限制条件
    float dt;// 增量时间
    float pv;// 前一次数据
    float sp;// 后一次数据
};

// PID转速控制模块初始化(每分钟转数 Revolutions Per Minute)
void rpmctl_init(void);
// 重置PID转速控制模块
void rpmctl_reset(void);
// 更新PID转速控制模块
float rpmctl_update(const struct rpmctl_input* input);
