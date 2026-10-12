#include "camera_driver.h"

#include <opencv2/opencv.hpp>

#include "board_config.h"

static cv::VideoCapture g_camera;

/**
 * @brief 初始化摄像头。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Camera_Init(void)
{
    if (g_camera.isOpened()) {
        return 0;
    }

    if (!g_camera.open(CAMERA_DEVICE_NAME)) {
        return -1;
    }

    g_camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    g_camera.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH);
    g_camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT);
    g_camera.set(cv::CAP_PROP_FPS, CAMERA_FPS);
    return 0;
}

/**
 * @brief 读取一帧摄像头图像。
 * @param frame 输出图像。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Camera_ReadFrame(cv::Mat *frame)
{
    if (frame == NULL || !g_camera.isOpened()) {
        return -1;
    }
    if (!g_camera.read(*frame)) {
        return -1;
    }
    return 0;
}

/**
 * @brief 释放摄像头。
 * @retval 无。
 */
void Camera_Release(void)
{
    g_camera.release();
}