#ifndef DRIVER_VEHICLE_COMM_H
#define DRIVER_VEHICLE_COMM_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief 上位机文本指令回调函数类型。
 * @param text 接收到的文本指令。
 * @retval 无。
 */
typedef void (*VehicleComm_CommandCallback)(const char *text);

int VehicleComm_Init(const char *host, uint16_t port);
int VehicleComm_Start(VehicleComm_CommandCallback callback);
void VehicleComm_Stop(void);
int VehicleComm_SendImage(const uint8_t *jpeg, size_t jpeg_len, const char *meta);
int VehicleComm_SendText(const char *text);

#endif /* DRIVER_VEHICLE_COMM_H */