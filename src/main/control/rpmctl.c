#include "include.h"

static struct state
{
    float integrated;
    float prev_error;
} _state;

static struct params
{
    float p;
    float d;
    float i;
} _params;

void rpmctl_init(void)
{
    _params.p = rpmctl_Config()->rpmctl_p;
    _params.d = rpmctl_Config()->rpmctl_d;
    _params.i = rpmctl_Config()->rpmctl_i;
}

void rpmctl_reset(void)
{
    _state.integrated = 0.0;
    _state.prev_error = nan("");
}

float rpmctl_update(const struct rpmctl_input* input)
{
    const float error = input->sp - input->pv;
    assert_param(isfinite(error));


    if (!isfinite(_state.prev_error)) {
        _state.prev_error = error;
    }

    const float p = error * _params.p;
    const float i = _state.integrated + error * input->dt * _params.i;
    const float d = ((error - _state.prev_error) / input->dt) * _params.d;

    _state.prev_error = error;

    float output = p + i + d;
    if (output > 1.0f) {
        output = 1.0f;
    } else if (output < -1.0f) {
        output = -1.0f;
    } else if (input->limit_mask == 0) {
        _state.integrated = i;
    }

    return output;
}
