#ifndef DRIVER_PERIODIC_TIMER_H
#define DRIVER_PERIODIC_TIMER_H

#include <stdint.h>

/**
 * @brief 周期定时器回调函数类型。
 */
typedef void (*PeriodicTimer_Callback)(void);

int PeriodicTimer_InitMs(uint32_t ms, PeriodicTimer_Callback callback);

#endif /* DRIVER_PERIODIC_TIMER_H */