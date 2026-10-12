#include "periodic_timer.h"

#include <functional>

#include "zf_driver_pit.h"

/**
 * @brief 初始化毫秒级周期定时器。
 * @param ms 周期，单位毫秒。
 * @param callback 周期回调函数。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PeriodicTimer_InitMs(uint32_t ms, PeriodicTimer_Callback callback)
{
    if (callback == NULL || ms == 0) {
        return -1;
    }
    std::function<void()> func = callback;
    pit_ms_init(ms, func);
    return 0;
}