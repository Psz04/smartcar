#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include "zf_common_headfile.h"
#include "lq_ncnn.h"
#include <opencv2/opencv.hpp>
#include "board_config.h"
#include "control.h"
#include "image.h"
#include "init.h"
#include "motor_driver.h"
#include "protocol.h"
#include "vehicle_comm.h"
#include "voltage_driver.h"

static volatile int g_program_running = 1; // 主函数运行标志

static volatile int g_transmit_fps = 10; // 图传目标帧率，单位 fps。默认 10，可由 SET_FPS 指令修改

/**
 * @brief Ctrl+C 信号处理函数。
 * @param signal_number 信号编号，本函数中不使用。
 * @retval 无。
 */
static void OnSignal(int signal_number)
{
    (void)signal_number;
    g_program_running = 0; // 主函数停止运行
    Control_SetRunState(0);
}

/**
 * @brief 上位机指令处理函数。
 * @param text 上位机发送的文本指令，格式为 TYPE, 参数1, 参数2, ...。
 */
static void OnCommand(const char *text)
{
    ProtocolCommand command;
    if(Protocol_ParseCommand(text, &command) != 0)
    {
        return; // 指令解析失败，不处理
    }

    printf("[Command] %s\n", text); // 打印接收到的指令

    if(strcmp(command.type, "START") == 0) // 发车指令
    {
        Control_SetRunState(1);
    }
    else if(strcmp(command.type, "STOP") == 0) // 停车指令
    {
        Control_SetRunState(0);
    }
    else if(strcmp(command.type, "PING") == 0) // 上位机连接测试指令
    {
        VehicleComm_SendText("PONG");
    }
    else if(strcmp(command.type, "BATTERY") == 0) // 电池电压查询指令
    {
        char msg[20] = {};
        sprintf(msg, "battery: %.2f", Voltage_Read());
        VehicleComm_SendText(msg);
    }
    else if(strcmp(command.type, "SET_FPS") == 0 && command.param_count >= 1) // 图传帧率设置指令
    {
        int fps = atoi(command.params[0]); // 字符串 -> 整数
        if (fps < 1)
        {
            fps = 1;
        }
        if (fps > 60)
        {
            fps = 60;
        }
        g_transmit_fps = fps;
    }
    else if(Control_HandleCommand(&command) != 0) // 处理与控制相关的指令
    {
        // Control_HandleCommand 已处理
    }
    else // 未知指令
    {
        VehicleComm_SendText("ERROR,UNKNOWN_COMMAND");
    }
}

/**
 * @brief 车端主函数。
 * @retval 0 正常退出。
 * @retval 1 初始化失败。
 */
int main(void)
{
    signal(SIGINT, OnSignal);

    if(systemInit(OnCommand, &g_program_running) != 0) // 系统初始化，包括开启控制中断和两个通信线程
    {
        printf("systemInit failed\n");
        return 1;
    }
    Control_SetServoPid(14, 0, 2);  // 设置舵机 PID
    Control_SetMotorBasePwm(2000);  // 设置电机开环 PWM 值

    int frame_index = 0; // 帧索引
    while(g_program_running != 0) // 主循环，受 g_program_running 控制
    {
        if(Image_Update() != 0) // 图像处理
        {
            usleep(1000); // 处理失败则 1ms 后重试
            continue;
        }

        Control_SetTurningCenter(Image_GetTurningCenter()); // 将图像处理得到的转向中心传递给控制算法

        // 应用 FPS 并传输图像到上位机
        int fps = g_transmit_fps;
        if(fps < 1)
        {
            fps = 1;
        }
        if(fps > 60)
        {
            fps = 60;
        }
        int divider = CAMERA_FPS / fps;
        if(divider < 1)
        {
            divider = 1;
        }
        if((frame_index % divider) == 0)
        {
            Image_DrawOverlay();
            // const cv::Mat *img = Image_GetOverlayFrame(); // 传输叠加绘制了图像算法信息的图像
            const cv::Mat *img = Image_GetGrayFrame(); // 传输原始灰度图
            // const cv::Mat *img = Image_GetGrayFrame(); // 传输二值化图像
            if(img != NULL && !img->empty())
            {
                std::vector<unsigned char> jpeg;
                cv::imencode(".jpg", *img, jpeg);
                char meta[128];
                if(Protocol_BuildImageMeta(Image_GetTurningCenter(), fps, meta, sizeof(meta)) >= 0)
                {
                    VehicleComm_SendImage(jpeg.data(), jpeg.size(), meta);
                }
            }
        }

        frame_index++;
        usleep(1000);
    }

    Motor_Stop();
    Image_Release();
    // VehicleComm_Stop(); // 有 bug
    std::exit(1);
    return 0;
}


// 模型环境测试代码
// 使用方法：
// 1. 把上面的内容注释掉
// 2. 把下面的内容取消注释
// 3. 编译下载
// 4. 在 root 下上传模型文件 model.ncnn.bin 和 model.ncnn.param，然后上传测试图像命名为 test.jpg
// 5. 运行，输出推理结果、置信度等内容则模型环境可正常运行

// #include <stdio.h>
// #include "lq_ncnn.h"

// // ==================== 模型信息读取 ====================
// // 模型路径：默认与可执行文件同目录，按需修改
// #define MODEL_PARAM_PATH "model.ncnn.param"
// #define MODEL_BIN_PATH   "model.ncnn.bin"
// #define MODEL_INPUT_WIDTH  64
// #define MODEL_INPUT_HEIGHT 64
// #define MODEL_TEST_IMAGE_PATH "test.jpg"   // 测试图片路径，默认与可执行文件同目录

// int main()
// {
//     printf("NCNN: 开始读取模型信息 ...\n");

//     LQ_NCNN ncnn;
//     ncnn.SetModelPath(MODEL_PARAM_PATH, MODEL_BIN_PATH);
//     ncnn.SetInputSize(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT);

//     if (!ncnn.LoadModel(MODEL_PARAM_PATH, MODEL_BIN_PATH))
//     {
//         printf("NCNN: 模型信息读取失败\n");
//         return -1;
//     }

//     printf("NCNN: 模型信息读取成功（层数/Blob数/输入输出见上方日志）\n");

//     // ==================== 图片推理 ====================
//     // 如需输出类别名称：ncnn.SetLabels({"class0", "class1", ...}); 需在 LoadModel 之前调用
//     cv::Mat image = cv::imread(MODEL_TEST_IMAGE_PATH);
//     if (image.empty())
//     {
//         printf("NCNN: 读取测试图片失败: %s\n", MODEL_TEST_IMAGE_PATH);
//         return -1;
//     }
//     printf("NCNN: 图片读取成功 (%dx%d)，开始推理 ...\n", image.cols, image.rows);

//     float confidence = 0.0f;
//     std::string result = ncnn.Infer(image, confidence);
//     if (result.empty())
//     {
//         printf("NCNN: 推理失败\n");
//         return -1;
//     }
//     printf("NCNN: 推理结果: %s, 置信度: %.4f\n", result.c_str(), confidence);

//     return 0;
// }
