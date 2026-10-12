#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

/*
 * 离线调试 I/O 接口。
 *
 * 图像输入输出不放在 main.cpp 中，保持 main.cpp 简洁；叠加绘制由 image.cpp 提供。
 * 使用前请在 src/cameratest.cpp 顶部修改 INPUT_DIR、OUTPUT_DIR 以及
 * image_process_start_idx / image_process_end_idx。
 */
bool ReadImage();
void OutputImage();

/* 当前处理的起始/结束帧索引，包含首尾。 */
extern int image_process_start_idx;
extern int image_process_end_idx;

/* 当前帧灰度图和二值图。 */
extern cv::Mat grayFrame;
extern cv::Mat binaryFrame;

#endif // CAMERA_H
