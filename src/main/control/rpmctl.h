#pragma once

typedef struct rpmctl_Config_s {
    double rpmctl_p;
    double rpmctl_i;
    double rpmctl_d; 
} rpmctl_Config_t;

struct rpmctl_input
{
    int limit_mask;
    float dt;
    float pv;
    float sp;
};

void rpmctl_init(void);
void rpmctl_reset(void);
float rpmctl_update(const struct rpmctl_input* input);
