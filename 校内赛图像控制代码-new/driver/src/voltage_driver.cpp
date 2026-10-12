#include "voltage_driver.h"

#include "board_config.h"
#include "zf_driver_adc.h"

/**
 * @brief 读取电池电压。
 * @retval >=0 电池电压，单位 V。
 * @retval -1.0 读取失败。
 */
double Voltage_Read(void)
{
    uint16 adc_raw = adc_convert(ADC_REG_PATH);
    float adc_scale = adc_get_scale(ADC_SCALE_PATH);

    if (adc_raw == 0 || adc_scale <= 0.0f) {
        return -1.0;
    }

    /* 与原 get_battery_vol 保持一致：adc_scale 按 mV/LSB 处理。 */
    double battery_mv =
        (double)adc_raw * (double)adc_scale * BATTERY_VOLTAGE_DIVIDER_RATIO;
    return battery_mv / 1000.0;
}