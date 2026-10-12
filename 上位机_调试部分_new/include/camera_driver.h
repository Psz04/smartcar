#ifndef DEBUG_CAMERA_DRIVER_H
#define DEBUG_CAMERA_DRIVER_H

#include <opencv2/core.hpp>

/*
 * 离线调试用的 camera_driver 桩。
 *
 * 车端 image.cpp 里定义了 Image_Init/Image_Update/Image_Release，
 * 它们会引用下面的 Camera_* 符号。离线调试时不会调用这些摄像头接口，
 * 但仍需要提供空实现，链接才能通过。
 */
int Camera_Init(void);
int Camera_ReadFrame(cv::Mat *frame);
void Camera_Release(void);

#endif // DEBUG_CAMERA_DRIVER_H