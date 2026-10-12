#include "init.h"

#include <stdio.h>
#include <unistd.h>

#include "board_config.h"
#include "control.h"
#include "encoder_driver.h"
#include "image.h"
#include "motor_driver.h"
#include "periodic_timer.h"
#include "servo_driver.h"

/**
 * @brief 初始化车端硬件、控制、通信、摄像头和周期中断。
 * @param command_callback 上位机指令回调函数。
 * @param program_running 主程序运行标志，用于在初始化重试期间响应 Ctrl+C。
 * @retval 0 初始化成功。
 * @retval -1 初始化失败。
 */
int systemInit(VehicleComm_CommandCallback command_callback, const volatile int *program_running)
{
    if(Motor_Init() != 0)
    {
        printf("Motor_Init failed\n");
        return -1;
    }

    if(Servo_Init() != 0)
    {
        printf("Servo_Init failed\n");
        return -1;
    }

    if(Encoder_DriverInit() != 0)
    {
        printf("Encoder_DriverInit failed\n");
    }

    Control_Init();

    if(VehicleComm_Init(UPPER_COMPUTER_IP, UPPER_COMPUTER_PORT) != 0)
    {
        printf("VehicleComm_Init failed\n");
        return -1;
    }

    if(VehicleComm_Start(command_callback) != 0)
    {
        printf("VehicleComm_Start failed\n");
        return -1;
    }

    while(Image_Init() != 0)
    {
        if(program_running == NULL || *program_running == 0)
        {
            return -1;
        }
        printf("Camera_Init failed, retry...\n");
        sleep(1);
    }

    if(program_running != NULL && *program_running == 0)
    {
        return -1;
    }

    if(PeriodicTimer_InitMs(CONTROL_PERIOD_MS, Control_Update5ms) != 0)
    {
        printf("PeriodicTimer_InitMs failed\n");
        return -1;
    }

    return 0;
}