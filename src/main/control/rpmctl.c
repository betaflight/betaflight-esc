#include "include.h"

static struct state
{
    float integrated;// 集成值
    float prev_error;// 前一次的误差
} _state;

static struct params
{
    float p;// P值
    float d;// D值
    float i;// I值
} _params;

/*
 * 转速控制模块初始化(每分钟转数 Revolutions Per Minute)
 */
void rpmctl_init(void)
{
    _params.p = rpmctl_Config()->rpmctl_p;
    _params.d = rpmctl_Config()->rpmctl_d;
    _params.i = rpmctl_Config()->rpmctl_i;
}

/*
 * 重置转速控制模块
 */
void rpmctl_reset(void)
{
    _state.integrated = 0.0;
    _state.prev_error = nan("");
}

/*
 * 更新转速控制模块
 */
float rpmctl_update(const struct rpmctl_input* input)
{
    // 误差值 = 后一次数据 - 前一次数据
    const float error = input->sp - input->pv;
    assert_param(isfinite(error));

    // 记录该项数据
    if (!isfinite(_state.prev_error)) {
        _state.prev_error = error;
    }

    // 比例项 = 误差 * 比例系数
    const float p = error * _params.p;
    // 积分项
    const float i = _state.integrated + error * input->dt * _params.i;
    // 微分项
    const float d = ((error - _state.prev_error) / input->dt) * _params.d;

    // 记录前次的误差值
    _state.prev_error = error;

    // 输出值
    float output = p + i + d;
    if (output > 1.0) {
        output = 1.0;// 正向限幅
    } else if (output < -1.0) {
        output = -1.0;// 负向限幅
    } else if (input->limit_mask == 0) {
        _state.integrated = i;
    }

    return output;
}
