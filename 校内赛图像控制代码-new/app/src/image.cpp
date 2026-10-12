#include "image.h"
#include <string.h>
#include <opencv2/imgproc.hpp>
#include "board_config.h"
#include "camera_driver.h"

uint8_t g_VideoImageData[MAX_VIDEO_LINE][MAX_VIDEO_POINT]; // 灰度图像数组
uint8_t binary_threshold = 130; // OTSU 阈值


int edge_num_l = 0; // 左边沿点数
ImagePoint edge_l[MAX_EDGE_POINT_NUM]; // 左边沿点数组
int edge_num_r = 0; // 右边沿点数
ImagePoint edge_r[MAX_EDGE_POINT_NUM]; // 右边沿点数组
int mid_point_num = 0; // 中线点数
ImagePoint mid_points[MAX_MID_POINT_NUM]; // 中线点数组

int turning_center = IMAGE_CENTER_X; // 转向中心，默认在图像正中间

static cv::Mat g_raw_frame; // 摄像头原始 BGR 图像
static cv::Mat g_gray_frame; // 灰度图
static cv::Mat g_binary_frame; // OTSU 二值化图像
static cv::Mat g_overlay_frame; // 带算法标注的调试图

// 为了代码简短的重命名宏定义
#define IMG                 g_VideoImageData        // 灰度图像数组
#define IMG_H               MAX_VIDEO_LINE          // 图像高度
#define IMG_W               MAX_VIDEO_POINT         // 图像宽度
#define MAX_WB_NUM          MAX_WHITE_BLOCK_NUM     // 每行最大白块数量
#define TURN_HEIGHT         TURNING_CALC_HEIGHT     // 转向中心计算高度

// 实用宏定义函数
#define IS_WHITE(x, y)      (IMG[(x)][(y)] > binary_threshold)
#ifndef MIN
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#endif

/**
 * @brief 图像处理初始化，清空上一轮的遗留变量
 */
static void variable_init(void)
{
    edge_num_l = 0;
    edge_num_r = 0;
    mid_point_num = 0;
}

/**
 * @brief 图像处理主函数。
 */
void Search()
{
    /* 
    运行到 Search() 时，新一帧图像已经被拷贝到 img 中。
    我们使用联通白块法逐行向上搜索。
    */
    variable_init();
    int line = 0;   // 当前搜索行
    
    // 逐行向上搜索
    for(line = 0; line < IMG_H; line++)
    {
        // 遍历本行，寻找本行所有的白块（连续的白色像素）
        int white_block_num = 0; // 本行白块数
        static uint8_t wb_start[MAX_WB_NUM] = {}; // 本行白块(们)的左侧起点
        static uint8_t wb_end[MAX_WB_NUM] = {}; // 本行白块(们)的右侧终点
        
        // 从左到右遍历，计算本行白块数，并填充起点和终点数组
        // 外层循环：找白块起点
        for(int i = 0; i < IMG_W && white_block_num < MAX_WB_NUM; )
        {
            // 如果是白色，说明找到起点
            if(IS_WHITE(line, i))
            {
                wb_start[white_block_num] = i;

                // 内层循环：找白块终点
                do 
                {
                    i++;
                }
                while(i < IMG_W && IS_WHITE(line, i));
                
                // 跳出内层循环时，i 已经指向黑色像素或右边界外，它的左侧才是白块终点
                wb_end[white_block_num] = i - 1;
                white_block_num++;
            }
            else
            {
                i++;
            }
        }

        // 单独处理第一行，这一行选择的白块是后续找联通白块的基础
        if(line == 0)
        {
            int base_block = 0; // 第一行白块下标
            if(white_block_num == 0)
            {
                // 如果第一行没有白块，则可以判定为出界
                return;
            }
            // 找最长的白块作为第一行白块
            int wb_width = wb_end[0] - wb_start[0]; // 最长白块宽度
            for(int i = 1; i < white_block_num; i++)
            {
                int temp = wb_end[i] - wb_start[i]; // 当前白块宽度
                if(temp > wb_width)
                {
                    wb_width = temp;
                    base_block = i;
                }
            }

            // 给第一行白块赋值
            edge_l[0].x = line;
            edge_l[0].y = wb_start[base_block];
            edge_num_l = 1;

            edge_r[0].x = line;
            edge_r[0].y = wb_end[base_block];
            edge_num_r = 1;
        }
        else
        {
            // 如果这一行没有白块，可能已经到头了，结束搜线
            if(white_block_num == 0)
            {
                break;
            }

            /*
            选择这一行白块的方法：由于图像白色区域是联通的，所以选择与上一行已选白块（边沿之间的部分）重叠覆盖的白块
            */

            int cover_length = 0; // 覆盖部分长度
            int cover_num = 0; // 覆盖白块段数
            static int cover_wb[MAX_WB_NUM] = {}; // 覆盖白块下标数组
            // 遍历当前行的白块，统计与上一行有覆盖的
            for(int i = 0; i < white_block_num; i++)
            {
                // 计算这一段白块的覆盖长度
                cover_length = MIN(wb_end[i], edge_r[edge_num_r - 1].y) - MAX(wb_start[i], edge_l[edge_num_l - 1].y) + 1;
                // 如果存在覆盖，则保存这段白块的下标
                if(cover_length > 0)
                {
                    cover_wb[cover_num] = i;
                    cover_num++;
                }
            }
            // 情况一：只有一个重叠白块，那这一行的边沿就用它
            if(cover_num == 1)
            {
                edge_l[edge_num_l].x = line;
                edge_l[edge_num_l].y = wb_start[cover_wb[0]];
                edge_num_l++;
                edge_r[edge_num_r].x = line;
                edge_r[edge_num_r].y = wb_end[cover_wb[0]];
                edge_num_r++;
            }
            // 情况二：没有重叠白块，说明已经找到头了，结束搜线
            else if(cover_num == 0)
            {
                break;
            }
            // 情况三：有多段重叠白块，需要取舍
            else
            {
                // TODO: 请自己实现一套取舍逻辑，此处仅简单取第一段重叠白块
                // 提示: 遍历重叠白块，用中点与上一行边沿中点距离最近的那一段
                edge_l[edge_num_l].x = line;
                edge_l[edge_num_l].y = wb_start[cover_wb[0]];
                edge_num_l++;
                edge_r[edge_num_r].x = line;
                edge_r[edge_num_r].y = wb_end[cover_wb[0]];
                edge_num_r++;
            }
        }
    }
    
    // TODO: 斑马线识别
    // 提示: 白块段数

    // TODO: 十字路口识别及处理
    // 提示: 将边沿横坐标跳变大的点当作十字路口拐点，通过拐点来判定、处理十字路口

    // 赛道中线计算
    // TODO: 请自行实现弯道优化等更优算法，此处仅简单取左右边沿中点

    mid_point_num = edge_num_l;
    for(int i = 0; i < mid_point_num; i++)
    {
        mid_points[i].x = edge_l[i].x;
        mid_points[i].y = (edge_l[i].y + edge_r[i].y) / 2;
    }
}

/**
 * @brief 根据 Search() 求出的 mid_points 计算转向控制中心
 * @note 使用加权平均法，以高度为权重，计算中线点横坐标的平均值
 */
void calcTurningCenter(void)
{
    if(mid_point_num == 0)
    {
        return;
    }

    int weight_sum = 0;
    int weighted_sum = 0;

    for(int i = 0; i < mid_point_num; i++)
    {
        // 限制计算行数
        if(mid_points[i].x > TURN_HEIGHT)
        {
            break;
        }
        // 以高度为权重，加一是为了在只有一个点时仍能求出有效转向中心
        int weight = mid_points[i].x + 1;
        weight_sum += weight;
        weighted_sum += weight * mid_points[i].y;
    }
    // 防止除 0
    if(weight_sum > 0)
    {
        turning_center = weighted_sum / weight_sum;
    }
}

/**
 * @brief 绘制图像算法运行信息。
 * @note 应在 Search() 和 calcTurningCenter() 之后调用。
 */
void Image_DrawOverlay(void)
{
    cv::Mat display;
    #ifdef DEBUG
    // cv::cvtColor(binaryFrame, display, cv::COLOR_GRAY2BGR); // 在二值化图像上绘制
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR); // 在灰度图上绘制
    #else
    // cv::cvtColor(g_binary_frame, display, cv::COLOR_GRAY2BGR); // 在二值化图像上绘制
    cv::cvtColor(g_gray_frame, display, cv::COLOR_GRAY2BGR); // 在灰度图上绘制
    #endif

    const cv::Scalar red(0, 0, 255);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar blue(255, 0, 0);

    // 绘制左边沿
    for(int i = 0; i < edge_num_l; i++)
    {
        const int px = edge_l[i].y;
        const int py = MAX_VIDEO_LINE - 1 - edge_l[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            cv::circle(display, cv::Point(px, py), 1, red, -1);
        }
    }

    // 绘制右边沿
    for(int i = 0; i < edge_num_r; i++)
    {
        const int px = edge_r[i].y;
        const int py = MAX_VIDEO_LINE - 1 - edge_r[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            cv::circle(display, cv::Point(px, py), 1, green, -1);
        }
    }

    // 绘制中线
    for(int i = 0; i < mid_point_num; i++)
    {
        const int px = mid_points[i].y;
        const int py = MAX_VIDEO_LINE - 1 - mid_points[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            cv::circle(display, cv::Point(px, py), 1, blue, -1);
        }
    }

    g_overlay_frame = display;
}

/**
 * @brief 获取绘制了算法运行信息的图像。
 * @retval 带标注的 BGR 图像指针，调用者只读使用，不要手动 release。
 */
const cv::Mat *Image_GetOverlayFrame(void)
{
    return &g_overlay_frame;
}

/**
 * @brief 初始化摄像头。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Image_Init(void)
{
    if(Camera_Init() != 0)
    {
        return -1;
    }
    turning_center = IMAGE_CENTER_X;
    return 0;
}

/**
 * @brief 采集一帧图像并完成图像处理。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Image_Update(void)
{
    // 采集图像
    if(Camera_ReadFrame(&g_raw_frame) != 0 || g_raw_frame.empty())
    {
        return -1;
    }

    // 定义 ROI，裁剪图像
    cv::Rect roi(CAMERA_ROI_X, CAMERA_ROI_Y, CAMERA_ROI_WIDTH, CAMERA_ROI_HEIGHT);

    // 范围检查，防止越界
    if(roi.x < 0 || roi.y < 0 || roi.x + roi.width > g_raw_frame.cols || roi.y + roi.height > g_raw_frame.rows)
    {
        return -1;
    }

    cv::Mat roi_image = g_raw_frame(roi);

    // 转换成灰度图
    cv::cvtColor(roi_image, g_gray_frame, cv::COLOR_BGR2GRAY);

    // 如果摄像头装反了可以取消下面的注释，用代码翻转图像
    cv::flip(g_gray_frame, g_gray_frame, -1);

    // OTSU 二值化
    double threshold = cv::threshold(g_gray_frame, g_binary_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    binary_threshold = (uint8_t)threshold;

    // 将灰度图像写入数组 g_VideoImageData
    for(int i = 0; i < MAX_VIDEO_LINE; i++)
    {
        const uint8_t *row = g_gray_frame.ptr<uchar>(MAX_VIDEO_LINE - 1 - i);
        for(int j = 0; j < MAX_VIDEO_POINT; j++)
        {
            g_VideoImageData[i][j] = row[j];
        }
    }
    // 调用图像处理算法
    Search();
    // 计算控制中心
    calcTurningCenter();
    return 0;
}

/**
 * @brief 获取当前转向中心。
 * @retval 转向中心。
 */
int Image_GetTurningCenter(void)
{
    return turning_center;
}

/**
 * @brief 获取灰度图像。
 * @retval 灰度图像指针，调用者只读使用，不要手动 delete 或 release
 */
const cv::Mat *Image_GetGrayFrame(void)
{
    return &g_gray_frame;
}

/**
 * @brief 获取二值图像。
 * @retval 二值图像指针，调用者只读使用，不要手动 delete 或 release
 */
const cv::Mat *Image_GetBinaryFrame(void)
{
    return &g_binary_frame;
}

/**
 * @brief 释放摄像头和图像缓存。
 * @retval 无。
 */
void Image_Release(void)
{
    Camera_Release();
    g_raw_frame.release();
    g_gray_frame.release();
    g_binary_frame.release();
    g_overlay_frame.release();
}
