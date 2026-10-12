#ifndef DEBUG_BOARD_CONFIG_H
#define DEBUG_BOARD_CONFIG_H

/*
 * 离线调试用的板级参数。
 *
 * 这里的相机参数必须与车端 driver/inc/board_config.h 保持一致；
 * 否则 image.cpp 中的 ROI、图像数组尺寸等会和车端不同。
 */
#include "debug_compat.h"

#define CAMERA_DEVICE_NAME              "/dev/video0"
#define CAMERA_FRAME_WIDTH              160
#define CAMERA_FRAME_HEIGHT             120
#define CAMERA_FPS                      60
#define CAMERA_ROI_X                    0
#define CAMERA_ROI_Y                    10
#define CAMERA_ROI_WIDTH                160
#define CAMERA_ROI_HEIGHT               97

#define IMAGE_CENTER_X                  80

#endif /* DEBUG_BOARD_CONFIG_H */