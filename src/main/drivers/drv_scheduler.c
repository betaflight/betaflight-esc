#include "include.h"

static struct Timer* head_handle = NULL;

static volatile uint32_t _timer_ticks = 0;

void timer_init(struct Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat)
{
    // memset(handle, sizeof(struct Timer), 0);
    handle->timeout_cb = timeout_cb;
    handle->timeout = _timer_ticks + timeout;
    handle->repeat = repeat;
}

int timer_start(struct Timer* handle)
{
    struct Timer* target = head_handle;
    while(target) {
        if(target == handle) {
            return -1;
        }
        target = target->next;
    }
    handle->next = head_handle;
    head_handle = handle;

    return 0;
}

void timer_stop(struct Timer* handle)
{
    struct Timer** curr;

    for(curr = &head_handle; *curr; ) {
        struct Timer* entry = *curr;
        if (entry == handle) {
            *curr = entry->next;
        } else {
            curr = &entry->next;
        }
    }
}

void timer_loop(void)
{
    struct Timer* target;

    for(target = head_handle; target; target = target->next) {
        if(_timer_ticks >= target->timeout) {
            if(target->repeat == 0) {
                timer_stop(target);
            } else {
                target->timeout = _timer_ticks + target->repeat;
            }
            target->timeout_cb();
        }
    }
}

void timer_ticks(void)
{
    _timer_ticks++;
}
