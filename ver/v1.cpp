#include "image.h"
#include <string.h>
#include <vector>
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

/* ---------------- 赛道元素识别结果 ---------------- */
int zebra_row_start = -1; // 斑马线起始行（-1 表示未识别到）
int zebra_row_end = -1;   // 斑马线结束行
int cross_row = -1;       // 十字路口拐点行（-1 表示未识别到）
int obstacle_found = 0;   // 是否识别到障碍块
int obstacle_row = -1;    // 障碍块所在行
int obstacle_col = -1;    // 障碍块中心列
int ring_found = 0;       // 是否识别到靶环
int ring_cx = -1;         // 靶环中心列
int ring_cy = -1;         // 靶环中心行
int ring_r = 0;           // 靶环半径

static cv::Mat g_raw_frame; // 摄像头原始 BGR 图像
static cv::Mat g_gray_frame; // 灰度图
static cv::Mat g_binary_frame; // OTSU 二值化图像
static cv::Mat g_overlay_frame; // 带算法标注的调试图

static int g_road_width = 60; // 赛道的估计宽度（列数），用于单边丢线时补齐

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
#ifndef ABS_OF
#define ABS_OF(v)           ((v) < 0 ? -(v) : (v))
#endif

/* ---------------- 算法参数（均经全序列 890 帧回放调参） ---------------- */
#define ROAD_SIDE_TOL        4     // 允许边沿逐行漂移的容差（列）
#define GAP_MERGE_MAX        6     // 相邻白块间隙 <= 此值视为赛道被"切碎"（斑马线黑条）
#define ZEBRA_MIN_ROWS       3     // 连续"碎块"行数 >= 此值判定为斑马线
#define ZEBRA_MIN_BLOCKS     3     // "碎块"行的白块段数 >= 此值
#define ZEBRA_WIDTH_MAX      1.25f // 斑马线所在行的赛道宽度不得超过正常宽度的多少倍

/* 正入十字：横向白臂会让赛道包络突然变宽。
   实测真值帧（226-274 / 378-435 / 790-838）的形态是：
   某一行宽度相对上一行突增 >=10 像素，且达到中位宽度的 1.35 倍以上，
   同时至少一侧边沿触到图像边界（横向臂一直伸到画面外）。 */
#define CROSS_JUMP_MIN       10    // 单行宽度突增的最小像素数
#define CROSS_WIDTH_RATIO    1.35f // 该行宽度相对"正常宽度"的最小倍数
#define CROSS_MIN_WIDTH      50    // 该行宽度的绝对下限（列）
#define CROSS_SAFE_TOP       4     // 距赛道搜索终点多少行以内不判定（避免"赛道到头"截断）
#define CROSS_EDGE_MARGIN    1     // 边沿距图像左右边界多少列以内算"触边"

/* 障碍块：黑块贴在赛道一侧，会把该侧边沿突然"啃"进去一块。
   要求：单侧内缩跳变 >=10 像素（另一侧基本不动），且此后连续多行保持该内缩量，
   以区别于弯道那种一次性折角。 */
#define OBSTACLE_JUMP        10    // 单侧边沿单行内缩的最小像素数
#define OBSTACLE_WDROP       14    // 或者：赛道宽度单行骤降的最小像素数
#define OBSTACLE_OTHER_TOL   4     // 另一侧边沿允许的最大位移
#define OBSTACLE_MIN_WIDTH   10    // 判定行的赛道宽度下限
#define OBSTACLE_PERSIST     6     // 跳变后需保持内缩的行数
#define OBSTACLE_PERSIST_FRAC 0.55f// "保持内缩"的判定倍数（相对跳变量）
#define OBSTACLE_SAFE_TOP    3     // 距搜索终点多少行以内不判定

/* 靶环：靶纸是赛道上一小片"亮底 + 细暗圆环"。
   OTSU 二值化后圆环几乎消失，必须在灰度图上找。
   关键判据是"暗像素/外接矩形"很小（细线），并且暗线围出内孔（有子轮廓）。
   注意不能用 cv::contourArea 判细：闭合细环的 contourArea 会退化成整个圆盘面积。 */
#define RING_ROW_MIN         8     // 靶环检测的行范围
#define RING_ROW_MAX         70
#define RING_DARK_OFFSET     14    // 暗环相对赛道均值的灰度下降量
#define RING_MIN_SIZE        12    // 靶环外接矩形的最小边长
#define RING_MAX_SIZE        52    // 靶环外接矩形的最大边长
#define RING_MIN_FILL        0.15f // 暗像素占比下限（太低是噪点）
#define RING_MAX_FILL        0.50f // 暗像素占比上限（实心暗块远大于它）
#define RING_MIN_ASPECT      0.6f  // 外接矩形长宽比范围（圆环应接近正方）
#define RING_MAX_ASPECT      1.8f
#define RING_NEED_HOLE       1     // 是否要求暗线闭合成环（有内孔）

#define FIT_WINDOW           22    // 十字补线时，用拐点下方多少行做最小二乘拟合
#define MID_FIT_DEV          20    // 中线点偏离拟合直线超过此值就按拟合值修正（野点抑制）

/* ---------------- 逐行扫描的中间结果 ----------------
   提到文件作用域，是为了让 detectTargetRing() 等元素识别函数直接复用，
   不必把整张扫描结果当参数层层传递。 */
static int g_row_ok[MAX_VIDEO_LINE];  // 本行赛道包络是否有效
static int g_row_l[MAX_VIDEO_LINE];   // 本行赛道左边界（列）
static int g_row_r[MAX_VIDEO_LINE];   // 本行赛道右边界（列）
static int g_top_line = -1;           // 成功搜索到的最高行号

/**
 * @brief 图像处理初始化，清空上一轮的遗留变量
 */
static void variable_init(void)
{
    edge_num_l = 0;
    edge_num_r = 0;
    mid_point_num = 0;

    zebra_row_start = -1;
    zebra_row_end = -1;
    cross_row = -1;
    obstacle_found = 0;
    obstacle_row = -1;
    obstacle_col = -1;
    ring_found = 0;
    ring_cx = -1;
    ring_cy = -1;
    ring_r = 0;
}

/**
 * @brief 取数组中位数（用于估计赛道宽度，避免被十字等异常行带偏）。
 */
static int median_of(int *values, int count)
{
    if(count <= 0)
    {
        return 0;
    }
    int sorted[IMG_H];
    if(count > IMG_H)
    {
        count = IMG_H;
    }
    memcpy(sorted, values, (size_t)count * sizeof(int));
    for(int i = 1; i < count; i++) // 插入排序，数据量很小
    {
        const int key = sorted[i];
        int j = i - 1;
        while(j >= 0 && sorted[j] > key)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[count / 2];
}

/**
 * @brief 用行号小于 from_row 的边沿点做最小二乘直线拟合，并把 from_row 之后的点按该直线外推。
 * @note  用于正入十字区域：拐点以上的边沿已经被横向赛道"污染"，不可信，
 *        因此用拐点下方的有效边沿拟合出赛道走向，再按该直线补齐，保证十字内直行。
 * @param edge 边沿点数组（按行号递增存放）。
 * @param num 边沿点个数。
 * @param from_row 从此行号开始（含）做外推。
 * @param fill_to 一直补齐到该行号（含）。
 */
static void extrapolate_edge(ImagePoint *edge, int *num, int from_row, int fill_to)
{
    if(num == NULL || *num < 4 || from_row < 3)
    {
        return;
    }

    // 用拐点下方最近 FIT_WINDOW 行做拟合，取最近的数据更能反映赛道当前的走向
    int begin = from_row - FIT_WINDOW;
    if(begin < 0)
    {
        begin = 0;
    }

    int n = 0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for(int i = 0; i < *num; i++)
    {
        if(edge[i].x < begin || edge[i].x >= from_row)
        {
            continue;
        }
        const double x = (double)edge[i].x;
        const double y = (double)edge[i].y;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        n++;
    }

    if(n < 3)
    {
        return;
    }

    const double den = (double)n * sxx - sx * sx;
    if(den <= 1e-6)
    {
        return;
    }

    const double ka = ((double)n * sxy - sx * sy) / den; // 斜率
    const double kb = (sy - ka * sx) / (double)n;        // 截距

    // 丢弃拐点及以上的原始点（它们来自横向赛道），只保留拐点以下的
    int keep = 0;
    while(keep < *num && edge[keep].x < from_row)
    {
        keep++;
    }

    // 按拟合直线，从拐点开始逐行补齐到 fill_to
    int line = from_row;
    while(line <= fill_to && keep < MAX_EDGE_POINT_NUM)
    {
        int value = (int)(ka * (double)line + kb + 0.5);
        if(value < 0)
        {
            value = 0;
        }
        if(value > IMG_W - 1)
        {
            value = IMG_W - 1;
        }
        edge[keep].x = (int16_t)line;
        edge[keep].y = (int16_t)value;
        keep++;
        line++;
    }
    *num = keep;
}

/**
 * @brief 靶环识别（在灰度图上做，OTSU 二值图上靶环基本不可见）。
 * @note  靶环的真实形态是"一小片亮底靶纸 + 上面一圈很细的暗圆环"。判据分三层：
 *          1) 在赛道内部（左右边沿各内缩 4 列以避开边界过渡带）统计灰度均值；
 *          2) 取低于均值 RING_DARK_OFFSET 的暗像素，再闭运算把断成几段的圆环连起来；
 *          3) 找最外层轮廓：外接矩形接近正方、尺寸适中、"暗像素 / 外接矩形" 很小
 *             （细线特征），并且暗线闭合成环（有子轮廓 = 有内孔）。
 * @attention 这里不能用 cv::contourArea 判断"细"：闭合细环的 contourArea 会退化成
 *            它围出的整个圆盘面积（约 0.7~0.8 倍外接矩形），与实心暗块（约 0.95）
 *            几乎分不开；必须改用真实暗像素计数（细环约 0.35~0.45）。
 */
static void detectTargetRing(void)
{
    ring_found = 0;
    ring_cx = -1;
    ring_cy = -1;
    ring_r = 0;

    const int row_begin = RING_ROW_MIN;
    const int row_end = MIN(RING_ROW_MAX, IMG_H);
    if(row_end - row_begin < 8 || g_top_line < row_begin + 8)
    {
        return;
    }

    // 构造赛道内部的灰度图，赛道以外填 0 作为掩膜
    cv::Mat road(row_end - row_begin, IMG_W, CV_8UC1, cv::Scalar(0));
    int road_pixels = 0;
    for(int x = row_begin; x < row_end; x++)
    {
        if(!g_row_ok[x])
        {
            continue;
        }
        int l = g_row_l[x] + 4; // 两侧各收缩 4 像素，避开赛道边界过渡带
        int r = g_row_r[x] - 4;
        if(r - l < 6)
        {
            continue;
        }
        if(l < 0)
        {
            l = 0;
        }
        if(r > IMG_W - 1)
        {
            r = IMG_W - 1;
        }
        for(int y = l; y <= r; y++)
        {
            road.at<uint8_t>(x - row_begin, y) = IMG[x][y];
            road_pixels++;
        }
    }

    if(road_pixels < 200)
    {
        return;
    }

    // 赛道内部的灰度均值（带掩膜，排除赛道外的 0）
    cv::Scalar mean_value, stddev_value;
    cv::meanStdDev(road, mean_value, stddev_value, road);
    const double road_mean = mean_value[0];
    if(road_mean < 60.0)
    {
        return;
    }

    // 暗像素 = 赛道内 且 灰度低于均值一定幅度
    const double dark_threshold = road_mean - RING_DARK_OFFSET;
    cv::Mat dark;
    cv::compare(road, cv::Scalar(dark_threshold), dark, cv::CMP_LT); // 小于阈值为 255
    cv::Mat road_mask;
    cv::compare(road, cv::Scalar(0), road_mask, cv::CMP_GT);
    cv::bitwise_and(dark, road_mask, dark);

    // 闭运算把断续的圆环连起来（靶环灰度对比度低，常被压成几段弧）
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dark, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int best_area = 0;
    for(size_t i = 0; i < contours.size(); i++)
    {
        if(hierarchy[i][3] != -1)
        {
            continue; // 只处理最外层轮廓
        }

        const cv::Rect box = cv::boundingRect(contours[i]);
        if(box.width < RING_MIN_SIZE || box.width > RING_MAX_SIZE)
        {
            continue;
        }
        if(box.height < RING_MIN_SIZE || box.height > RING_MAX_SIZE)
        {
            continue;
        }

        const double aspect = (double)box.width / (double)box.height;
        if(aspect < RING_MIN_ASPECT || aspect > RING_MAX_ASPECT)
        {
            continue; // 圆环的外接矩形应大致是正方形
        }

        const double box_area = (double)box.width * (double)box.height;
        if(box_area <= 0.0)
        {
            continue;
        }

#if RING_NEED_HOLE
        // 暗线闭合成环：有子轮廓说明环内是亮的（空心），实心暗块没有子轮廓
        if(hierarchy[i][2] == -1)
        {
            continue;
        }
#endif

        // 细线特征：暗像素只占外接矩形的一小部分
        const int dark_px = cv::countNonZero(dark(box));
        const double fill = (double)dark_px / box_area;
        if(fill < RING_MIN_FILL || fill > RING_MAX_FILL)
        {
            continue;
        }

        // 环心应当是亮的（排除实心暗块；同时避开掩膜外的 0）
        const cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
        const int probe = MIN(box.width, box.height) / 4;
        if(probe < 2)
        {
            continue;
        }
        const int py = center.y;
        const int px = center.x;
        if(py - probe < 0 || py + probe >= road.rows || px - probe < 0 || px + probe >= road.cols)
        {
            continue;
        }
        double inside = (double)road.at<uint8_t>(py, px);
        int samples = 1;
        const int dy4[4] = {-probe, probe, 0, 0};
        const int dx4[4] = {0, 0, -probe, probe};
        for(int k = 0; k < 4; k++)
        {
            const uint8_t v = road.at<uint8_t>(py + dy4[k], px + dx4[k]);
            if(v > 0)
            {
                inside += (double)v;
                samples++;
            }
        }
        if(inside / (double)samples < dark_threshold)
        {
            continue; // 中心也是黑的 -> 实心块，不是靶环
        }

        // 多个候选时取最大的那个：靶环越近，外接矩形越大
        if(box.width * box.height > best_area)
        {
            best_area = box.width * box.height;
            ring_found = 1;
            ring_cx = box.x + box.width / 2;
            ring_cy = box.y + box.height / 2 + row_begin; // IMG 行号（0 在图像最下面）
            ring_r = (box.width + box.height) / 4;
        }
    }
}

/**
 * @brief 图像处理主函数。
 * @note  运行到 Search() 时，新一帧图像已经被拷贝到 g_VideoImageData 中。
 *        算法流程：
 *          1) 自下而上逐行提取白块，用"与上一行赛道区间有重叠"的规则连通赛道；
 *          2) 记录内部黑洞（用于区分斑马线 / 障碍块）与边沿跳变（用于十字）；
 *          3) 正入十字：判定拐点 -> 拐点以上用最小二乘直线外推补线；
 *          4) 弯道中线：单边丢线用另一侧 + 赛道宽度补齐 -> 野点剔除 -> 滑动平均 -> 直线拟合。
 */
void Search()
{
    variable_init();

    static int row_blocks[MAX_VIDEO_LINE]; // 本行白块段数
    static int row_gapmax[MAX_VIDEO_LINE]; // 本行相邻白块的最大间隙
    static int row_gap_l[MAX_VIDEO_LINE];  // 本行最大黑洞的左侧列
    static int row_gap_r[MAX_VIDEO_LINE];  // 本行最大黑洞的右侧列
    static int width_hist[IMG_H];          // 各行赛道宽度，用于取中位数

    // 赛道包络结果直接落在文件作用域数组上，元素识别函数（如 detectTargetRing）要复用
    int *row_l = g_row_l;
    int *row_r = g_row_r;
    int *row_ok = g_row_ok;

    int line = 0;       // 当前搜索行
    int top_line = -1;  // 成功搜到的最高行
    int width_cnt = 0;  // width_hist 的有效长度

    // 逐行向上搜索
    for(line = 0; line < IMG_H; line++)
    {
        row_blocks[line] = 0;
        row_gapmax[line] = 0;
        row_l[line] = 0;
        row_r[line] = 0;
        row_ok[line] = 0;
        row_gap_l[line] = -1;
        row_gap_r[line] = -1;

        /* ---------- 1. 遍历本行，找出所有的白块（连续的白色像素） ---------- */
        int white_block_num = 0;
        static int wb_start[MAX_WB_NUM] = {};
        static int wb_end[MAX_WB_NUM] = {};

        for(int i = 0; i < IMG_W && white_block_num < MAX_WB_NUM; )
        {
            if(IS_WHITE(line, i))
            {
                wb_start[white_block_num] = i;
                do
                {
                    i++;
                }
                while(i < IMG_W && IS_WHITE(line, i));
                // 跳出内层循环时 i 已指向黑色像素或右边界外，其左侧才是白块终点
                wb_end[white_block_num] = i - 1;
                white_block_num++;
            }
            else
            {
                i++;
            }
        }

        row_blocks[line] = white_block_num;

        // 单独处理第一行，这一行选择的白块是后续找连通白块的基础
        if(line == 0)
        {
            if(white_block_num == 0)
            {
                return; // 第一行没有白块，判定为出界
            }
            int base_block = 0;
            int wb_width = wb_end[0] - wb_start[0];
            for(int i = 1; i < white_block_num; i++)
            {
                int temp = wb_end[i] - wb_start[i];
                if(temp > wb_width)
                {
                    wb_width = temp;
                    base_block = i;
                }
            }
            row_l[0] = wb_start[base_block];
            row_r[0] = wb_end[base_block];
            row_ok[0] = 1;

            edge_l[0].x = 0;
            edge_l[0].y = (int16_t)row_l[0];
            edge_num_l = 1;
            edge_r[0].x = 0;
            edge_r[0].y = (int16_t)row_r[0];
            edge_num_r = 1;

            width_hist[width_cnt++] = row_r[0] - row_l[0];
            top_line = 0;
            continue;
        }

        // 这一行没有白块，说明已经到头了，结束搜线
        if(white_block_num == 0)
        {
            break;
        }

        /* ---------- 2. 选择本行赛道区间 ----------
           图像中白色赛道是联通的，因此只保留与上一行赛道区间有重叠的白块。
           注意这里取的是这些白块的整体"包络"（最左起点 ~ 最右终点）：
           包络才是赛道真实的左右边界，赛道内部的黑洞（障碍块）不该让边界缩进去。 */
        const int prev_l = row_l[line - 1];
        const int prev_r = row_r[line - 1];

        int cover_idx[MAX_WB_NUM];
        int cover_num = 0;
        for(int i = 0; i < white_block_num; i++)
        {
            const int cover = MIN(wb_end[i], prev_r) - MAX(wb_start[i], prev_l) + 1;
            if(cover > 0 || (wb_start[i] <= prev_r + ROAD_SIDE_TOL && wb_end[i] >= prev_l - ROAD_SIDE_TOL))
            {
                cover_idx[cover_num] = i;
                cover_num++;
            }
        }

        if(cover_num == 0)
        {
            break; // 赛道断开，结束搜线
        }

        const int env_l = wb_start[cover_idx[0]];
        const int env_r = wb_end[cover_idx[cover_num - 1]];

        // 统计包络内部的间隙：找出最宽的那个黑洞（斑马线是很多窄缝，障碍块是一个宽洞）
        int gap_max = 0;
        int gap_l = -1;
        int gap_r = -1;
        for(int k = 1; k < cover_num; k++)
        {
            const int a = cover_idx[k - 1];
            const int b = cover_idx[k];
            const int gap = wb_start[b] - wb_end[a] - 1;
            if(gap > gap_max)
            {
                gap_max = gap;
                gap_l = wb_end[a];
                gap_r = wb_start[b];
            }
        }
        row_gapmax[line] = gap_max;
        row_gap_l[line] = gap_l;
        row_gap_r[line] = gap_r;

        /* ---------- 3. 正入十字拐点检测 ----------
           正入十字（车头正对十字、直行进入）时，横向白臂是"横插"进画面的，
           因此赛道包络会在某一行突然变宽。四个条件同时满足才算拐点：
             ① 单行宽度突增 >= CROSS_JUMP_MIN（逐行对比，跳变必须"突然"）；
             ② 该行宽度达到上一行的 CROSS_WIDTH_RATIO 倍以上；
             ③ 宽度绝对量 >= CROSS_MIN_WIDTH（避免窄视野下的噪声跳变）；
             ④ 至少一侧边沿一直伸到图像边界（横向臂伸出了画面）。
           条件④是关键：普通弯道虽然也会变宽，但边沿通常不会同时贴上画面边界。
           此外还要用"距搜索终点至少 CROSS_SAFE_TOP 行"做二次校验（见第 4 步之后）。 */
        if(cross_row < 0 && line >= 4)
        {
            const int width_ref = median_of(width_hist, width_cnt);
            const int width_now = env_r - env_l;
            const int width_prev = prev_r - prev_l;
            if(width_ref > 20
               && width_now >= CROSS_MIN_WIDTH
               && (width_now - width_prev) >= CROSS_JUMP_MIN
               && width_prev >= 12
               && width_now > (int)((float)width_prev * CROSS_WIDTH_RATIO)
               && (env_l <= CROSS_EDGE_MARGIN || env_r >= IMG_W - 1 - CROSS_EDGE_MARGIN))
            {
                cross_row = line; // 记录拐点行
            }
        }


        row_l[line] = env_l;
        row_r[line] = env_r;
        row_ok[line] = 1;

        edge_l[edge_num_l].x = (int16_t)line;
        edge_l[edge_num_l].y = (int16_t)env_l;
        edge_num_l++;

        edge_r[edge_num_r].x = (int16_t)line;
        edge_r[edge_num_r].y = (int16_t)env_r;
        edge_num_r++;

        width_hist[width_cnt++] = env_r - env_l;
        top_line = line;
    }

    /* ---------- 4. 补一步：主赛道前方仍存在横向白臂 ----------
       有时横向赛道与主赛道之间隔着一段黑色，逐行搜索会提前结束，
       此时主动检查搜线终点之上是否还有"横跨图像中部的宽白块"，有则说明前面是十字。 */
    if(cross_row < 0 && top_line >= 0 && (top_line + 2) < IMG_H)
    {
        int bar_rows = 0;
        for(int x = top_line + 1; x < IMG_H; x++)
        {
            int best_width = 0;
            int best_start = 0;
            int best_end = 0;
            for(int i = 0; i < IMG_W; )
            {
                if(IS_WHITE(x, i))
                {
                    int j = i;
                    while(j < IMG_W && IS_WHITE(x, j))
                    {
                        j++;
                    }
                    if((j - i) > best_width)
                    {
                        best_width = j - i;
                        best_start = i;
                        best_end = j - 1;
                    }
                    i = j;
                }
                else
                {
                    i++;
                }
            }
            if(best_width >= IMG_W * 45 / 100 && best_start < IMG_W / 4 && best_end > IMG_W * 3 / 4)
            {
                bar_rows++;
            }
        }
        if(bar_rows >= 2)
        {
            cross_row = top_line; // 拐点取主赛道顶部
        }
    }

    /* ---------- 4.5 拐点有效性校验 ----------
       搜索终点附近本来就是"赛道到头"的截断，那里宽度也会突变，必须排除；
       另外 top_line 太低（几乎没搜到赛道）时任何元素识别都不可信。 */
    if(cross_row >= 0
       && (top_line < 20 || cross_row > top_line - CROSS_SAFE_TOP))
    {
        cross_row = -1;
    }

    /* ---------- 5. 十字区域边界补线 ----------
       拐点以上的边沿来自横向赛道，不能再用；用拐点下方的有效边沿做最小二乘拟合，
       再按该直线把边沿一路补齐到图像顶部，使十字路口内中线保持平直（直行通过）。 */
    if(cross_row > 3)
    {
        extrapolate_edge(edge_l, &edge_num_l, cross_row, IMG_H - 1);
        extrapolate_edge(edge_r, &edge_num_r, cross_row, IMG_H - 1);
    }

    /* ---------- 6. 估计赛道宽度 ---------- */
    {
        const int width_ref = median_of(width_hist, width_cnt);
        if(width_ref > 8)
        {
            g_road_width = width_ref;
        }
    }

    /* ---------- 7. 斑马线识别 ----------
       斑马线是赛道上的黑色细横条，会把某一行切成很多小段白块（段间间隙很小），
       而赛道整体宽度不变，因此"连续多行白块段数骤增 + 间隙很小 + 宽度正常"即为斑马线。 */
    {
        int run_start = -1;
        for(int x = 0; x <= IMG_H; x++)
        {
            int is_fragment = 0;
            if(x < IMG_H && row_ok[x]
               && row_blocks[x] >= ZEBRA_MIN_BLOCKS
               && row_gapmax[x] > 0
               && row_gapmax[x] <= GAP_MERGE_MAX
               && (row_r[x] - row_l[x]) <= (int)((float)g_road_width * ZEBRA_WIDTH_MAX))
            {
                is_fragment = 1;
            }

            if(is_fragment)
            {
                if(run_start < 0)
                {
                    run_start = x;
                }
            }
            else if(run_start >= 0)
            {
                if((x - run_start) >= ZEBRA_MIN_ROWS)
                {
                    zebra_row_start = run_start;
                    zebra_row_end = x - 1;
                }
                run_start = -1;
            }
        }
    }

    /* ---------- 8. 障碍块识别 ----------
       障碍块是贴在赛道一侧的黑色方块，会把那一侧的边沿突然"啃"进去一块。
       实测形态（720-758 段）：单侧边沿在某一行突然内缩 10~15 像素，另一侧几乎不动，
       且此后连续多行都保持这个内缩量 —— 这正是"一个有限大的方块挡在边沿上"的特征，
       与弯道（边沿连续平滑变化）、十字（两侧同时外扩）都不相同。
       因此判据 = 单侧内缩跳变 + 跳变后持续内缩，二者缺一不可。 */
    if(top_line > OBSTACLE_SAFE_TOP + 2)
    {
        const int obs_limit = top_line - OBSTACLE_SAFE_TOP;
        for(int x = 2; x <= obs_limit; x++)
        {
            if(!row_ok[x] || !row_ok[x - 1])
            {
                continue;
            }

            const int dl = row_l[x] - row_l[x - 1]; // >0：左边界向右内缩
            const int dr = row_r[x] - row_r[x - 1]; // <0：右边界向左内缩
            const int w_now = row_r[x] - row_l[x];
            const int w_prev = row_r[x - 1] - row_l[x - 1];
            if(w_now < OBSTACLE_MIN_WIDTH)
            {
                continue;
            }

            int side = 0; // +1 左侧被挡，-1 右侧被挡，2 宽度骤降（分不出侧）
            if(dl >= OBSTACLE_JUMP && dr >= -OBSTACLE_OTHER_TOL)
            {
                side = 1;
            }
            else if(dr <= -OBSTACLE_JUMP && dl <= OBSTACLE_OTHER_TOL)
            {
                side = -1;
            }
            else if((w_prev - w_now) >= OBSTACLE_WDROP)
            {
                side = 2;
            }
            if(side == 0)
            {
                continue;
            }

            // 持续性校验：跳变后该侧要继续保持内缩（相对跳变前的趋势线）
            if(side != 2)
            {
                const int base = (side == 1) ? row_l[x - 1] : row_r[x - 1];
                double slope = 0.0;
                if(x >= 6)
                {
                    slope = (double)(base - ((side == 1) ? row_l[x - 6] : row_r[x - 6])) / 6.0;
                }
                int persist_cnt = 0;
                for(int k = x + 1; k <= obs_limit && k <= x + 8; k++)
                {
                    if(!row_ok[k])
                    {
                        break;
                    }
                    const int cur = (side == 1) ? row_l[k] : row_r[k];
                    const double pred = (double)base + slope * (double)(k - (x - 1));
                    const double dev = (side == 1) ? ((double)cur - pred) : (pred - (double)cur);
                    if(dev >= (double)OBSTACLE_JUMP * OBSTACLE_PERSIST_FRAC)
                    {
                        persist_cnt++;
                    }
                }
                if(persist_cnt < OBSTACLE_PERSIST)
                {
                    continue;
                }
            }

            obstacle_found = 1;
            obstacle_row = x; // IMG 行号，0 在图像最下面
            obstacle_col = (side == 1) ? row_l[x]
                          : ((side == -1) ? row_r[x] : (row_l[x] + row_r[x]) / 2);
            break; // 只报最靠近车的那一个
        }
    }

    /* ---------- 9. 弯道中线拟合 ----------
       单侧边沿跑出图像时，用另一侧边沿 + 估计的赛道宽度把丢失的一侧补回来，
       避免中线被错误地拉向一侧（这是弯道不"画龙"的关键）。 */
    mid_point_num = edge_num_l;
    if(mid_point_num > edge_num_r)
    {
        mid_point_num = edge_num_r;
    }
    if(mid_point_num > MAX_MID_POINT_NUM)
    {
        mid_point_num = MAX_MID_POINT_NUM;
    }

    for(int i = 0; i < mid_point_num; i++)
    {
        int l = edge_l[i].y;
        int r = edge_r[i].y;
        const int width = r - l;

        if(l <= 1 && width < g_road_width * 3 / 5)
        {
            // 左边沿贴到图像左边界：用右侧边沿补出左边沿
            l = r - g_road_width;
            if(l < 0)
            {
                l = 0;
            }
        }
        else if(r >= IMG_W - 2 && width < g_road_width * 3 / 5)
        {
            // 右边沿贴到图像右边界：用左侧边沿补出右边沿
            r = l + g_road_width;
            if(r > IMG_W - 1)
            {
                r = IMG_W - 1;
            }
        }

        mid_points[i].x = edge_l[i].x;
        mid_points[i].y = (int16_t)((l + r) / 2);
    }

    // 9.1 野点剔除：连续三个点中中间那个偏离前后均值太多，就用均值替换
    if(mid_point_num >= 5)
    {
        for(int i = 1; i < mid_point_num - 1; i++)
        {
            const int avg = ((int)mid_points[i - 1].y + (int)mid_points[i + 1].y) / 2;
            if(ABS_OF((int)mid_points[i].y - avg) > MID_FIT_DEV * 3 / 4)
            {
                mid_points[i].y = (int16_t)avg;
            }
        }
    }

    // 9.2 五点滑动平均，抑制个别跳变点带来的中线抖动
    if(mid_point_num >= 5)
    {
        static ImagePoint smoothed[MAX_MID_POINT_NUM];
        for(int i = 0; i < mid_point_num; i++)
        {
            int sum = 0;
            int cnt = 0;
            for(int k = i - 2; k <= i + 2; k++)
            {
                if(k < 0 || k >= mid_point_num)
                {
                    continue;
                }
                sum += mid_points[k].y;
                cnt++;
            }
            smoothed[i] = mid_points[i];
            smoothed[i].y = (int16_t)(sum / cnt);
        }
        for(int i = 0; i < mid_point_num; i++)
        {
            mid_points[i] = smoothed[i];
        }
    }

    // 9.3 中线最小二乘拟合：对参与转向计算的行拟合一条直线，
    //     偏离该直线过远的点视为野点，用拟合值替换，保证弯道处中线连续、可微。
    {
        const int fit_num = MIN(mid_point_num, TURN_HEIGHT + 1);
        int n = 0;
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for(int i = 0; i < fit_num; i++)
        {
            const double x = (double)mid_points[i].x;
            const double y = (double)mid_points[i].y;
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            n++;
        }
        const double den = (double)n * sxx - sx * sx;
        if(n >= 4 && den > 1e-6)
        {
            const double ka = ((double)n * sxy - sx * sy) / den;
            const double kb = (sy - ka * sx) / (double)n;
            for(int i = 0; i < mid_point_num; i++)
            {
                const double fit = ka * (double)mid_points[i].x + kb;
                if(ABS_OF((double)mid_points[i].y - fit) > MID_FIT_DEV)
                {
                    int value = (int)(fit + 0.5);
                    if(value < 0)
                    {
                        value = 0;
                    }
                    if(value > IMG_W - 1)
                    {
                        value = IMG_W - 1;
                    }
                    mid_points[i].y = (int16_t)value;
                }
            }
        }
    }

    /* ---------- 10. 靶环识别（灰度图上做，二值图上靶环基本不可见） ---------- */
    g_top_line = top_line; // 元素识别函数要复用搜索终点
    detectTargetRing();
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
    // 离线调试：当前帧由 cameratest.cpp 的 ReadImage() 放进 grayFrame，
    // g_gray_frame 只在 Image_Update() 里赋值（离线不调用），不能用它。
    if(grayFrame.empty())
    {
        return;
    }
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR); // 在灰度图上绘制
#else
    if(g_gray_frame.empty())
    {
        return;
    }
    cv::cvtColor(g_gray_frame, display, cv::COLOR_GRAY2BGR); // 在灰度图上绘制
#endif

    const cv::Scalar red(0, 0, 255);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar blue(255, 0, 0);
    const cv::Scalar yellow(0, 255, 255);
    const cv::Scalar magenta(255, 0, 255);
    const cv::Scalar cyan(255, 255, 0);
    const cv::Scalar orange(0, 165, 255); // 十字补线段专用色（BGR）

    // 绘制左边沿（十字拐点以上是"最小二乘外推补出来"的段，用橙色区分）
    for(int i = 0; i < edge_num_l; i++)
    {
        const int px = edge_l[i].y;
        const int py = MAX_VIDEO_LINE - 1 - edge_l[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            const bool is_fill = (cross_row >= 0 && edge_l[i].x > cross_row);
            cv::circle(display, cv::Point(px, py), 1, is_fill ? orange : red, -1);
        }
    }

    // 绘制右边沿
    for(int i = 0; i < edge_num_r; i++)
    {
        const int px = edge_r[i].y;
        const int py = MAX_VIDEO_LINE - 1 - edge_r[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            const bool is_fill = (cross_row >= 0 && edge_r[i].x > cross_row);
            cv::circle(display, cv::Point(px, py), 1, is_fill ? orange : green, -1);
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

    // 斑马线识别结果：黄线标出起止行 + 文字标签
    if(zebra_row_start >= 0 && zebra_row_end >= 0)
    {
        const int py1 = MAX_VIDEO_LINE - 1 - zebra_row_start;
        const int py2 = MAX_VIDEO_LINE - 1 - zebra_row_end;
        if(py1 >= 0 && py1 < display.rows)
        {
            cv::line(display, cv::Point(0, py1), cv::Point(display.cols - 1, py1), yellow, 1);
        }
        if(py2 >= 0 && py2 < display.rows)
        {
            cv::line(display, cv::Point(0, py2), cv::Point(display.cols - 1, py2), yellow, 1);
        }
        int ty = py1 - 3;
        if(ty < 8)
        {
            ty = py2 + 10;
        }
        if(ty > 8 && ty < display.rows)
        {
            cv::putText(display, "ZEBRA", cv::Point(2, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.32, yellow, 1, cv::LINE_AA);
        }
    }

    // 十字路口拐点（青色横线）：拐点以上用橙色外推补线，文字标签标出
    if(cross_row >= 0)
    {
        const int py = MAX_VIDEO_LINE - 1 - cross_row;
        if(py >= 0 && py < display.rows)
        {
            cv::line(display, cv::Point(0, py), cv::Point(display.cols - 1, py), cyan, 1);
            int ty = py - 3;
            if(ty < 8)
            {
                ty = py + 10;
            }
            if(ty > 8 && ty < display.rows)
            {
                cv::putText(display, "CROSS", cv::Point(2, ty),
                            cv::FONT_HERSHEY_SIMPLEX, 0.32, cyan, 1, cv::LINE_AA);
            }
        }
    }

    // 障碍块位置（品红十字 + 标签）
    if(obstacle_found && obstacle_row >= 0 && obstacle_col >= 0)
    {
        const int px = obstacle_col;
        const int py = MAX_VIDEO_LINE - 1 - obstacle_row;
        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            cv::drawMarker(display, cv::Point(px, py), magenta, cv::MARKER_CROSS, 9, 1);
            int ty = py - 6;
            if(ty < 8)
            {
                ty = py + 12;
            }
            if(ty > 8 && ty < display.rows)
            {
                cv::putText(display, "BLOCK", cv::Point(px - 14, ty),
                            cv::FONT_HERSHEY_SIMPLEX, 0.32, magenta, 1, cv::LINE_AA);
            }
        }
    }

    // 靶环识别结果：按考核要求在图像左上角用 cv::circle 画圆表示识别成功，
    // 同时在靶环真实位置再描一个细圈，便于在视频里看清识别效果。
    if(ring_found)
    {
        cv::circle(display, cv::Point(18, 18), 10, yellow, 2, cv::LINE_AA);
        cv::circle(display, cv::Point(18, 18), 2, yellow, -1, cv::LINE_AA);
        cv::putText(display, "RING", cv::Point(31, 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32, yellow, 1, cv::LINE_AA);

        const int rx = ring_cx;
        const int ry = MAX_VIDEO_LINE - 1 - ring_cy;
        if(rx > 2 && rx < display.cols - 2 && ry > 2 && ry < display.rows - 2)
        {
            cv::circle(display, cv::Point(rx, ry), ring_r > 2 ? ring_r : 6,
                       yellow, 1, cv::LINE_AA);
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
