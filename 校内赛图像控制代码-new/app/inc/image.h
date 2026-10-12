#ifndef APP_IMAGE_H
#define APP_IMAGE_H

#include <stdint.h>
#include "board_config.h"

#include <opencv2/core.hpp>

#define MAX_VIDEO_POINT         160     // 图像宽度
#define MAX_VIDEO_LINE          97      // 实际用于算法的图像高度
#define MAX_EDGE_POINT_NUM      127     // 左右边沿数组最大长度
#define MAX_MID_POINT_NUM       127     // 中线点数组最大长度
#define MAX_WHITE_BLOCK_NUM     10      // 每一行最多统计的白块数量
#define TURNING_CALC_HEIGHT     50      // 计算转向中心时使用的最大行号

/**
 * @brief 图像点结构
 */
typedef struct {
    int16_t x;   // 行号，0 为图像最下面一行
    int16_t y;   // 列号，0 为图像最左边
} ImagePoint;

extern uint8_t g_VideoImageData[MAX_VIDEO_LINE][MAX_VIDEO_POINT];
extern uint8_t binary_threshold;
extern int edge_num_l;
extern ImagePoint edge_l[MAX_EDGE_POINT_NUM];
extern int edge_num_r;
extern ImagePoint edge_r[MAX_EDGE_POINT_NUM];
extern int mid_point_num;
extern ImagePoint mid_points[MAX_MID_POINT_NUM];
extern int turning_center;

void Search(void);
void calcTurningCenter(void);
int Image_Init(void);
int Image_Update(void);
int Image_GetTurningCenter(void);
const cv::Mat *Image_GetGrayFrame(void);
const cv::Mat *Image_GetBinaryFrame(void);
void Image_DrawOverlay(void);
const cv::Mat *Image_GetOverlayFrame(void);
void Image_Release(void);

#endif /* APP_IMAGE_H */
