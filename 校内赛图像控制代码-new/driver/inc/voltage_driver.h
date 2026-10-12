#ifndef DRIVER_VOLTAGE_DRIVER_H
#define DRIVER_VOLTAGE_DRIVER_H

/**
 * @brief 读取电池电压。
 * @retval >=0 电池电压，单位 V。
 * @retval -1.0 读取失败。
 */
double Voltage_Read(void);

#endif /* DRIVER_VOLTAGE_DRIVER_H */