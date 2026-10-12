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

/* ---------------- 算法参数（均经全序列 885 帧回放调参） ---------------- */
#define ROAD_SIDE_TOL        4     // 允许边沿逐行漂移的容差（列）
#define GAP_MERGE_MAX        6     // 相邻白块间隙 <= 此值视为赛道被"切碎"（斑马线黑条）
#define ZEBRA_MERGE_GAP      14    // 首行"碎块合并"允许的最大间隙：斑马线黑条实际宽 7~9 列
#define ZEBRA_MIN_ROWS       3     // 连续"碎块"行数 >= 此值判定为斑马线
#define ZEBRA_MIN_BLOCKS     3     // "碎块"行的白块段数 >= 此值
#define ZEBRA_PEAK_BLOCKS    5     // 一串碎块行里最密一行的段数 >= 此值
#define ZEBRA_WIDTH_MAX      1.25f // 斑马线所在行的赛道宽度不得超过正常宽度的多少倍

/* 正入十字：改用"四个角点 + 贴边"判定，不再看"宽度是否突变"。
   实测形态（228—270 / 380—431 两段真值）：
     十字的横向白臂是向左、右两侧一起甩出去的，因此左、右边界会在几乎同一行
     各自向外"折"出一个角（左下角点 / 右下角点），折完之后边界一直伸到画面外（贴边），
     中间是一段几乎横跨整幅画面的宽白区。
   反例（0—72 帧的远弯）：只有一侧边界向外扩，另一侧是连续平滑的弯曲，
     左右角点不可能同时成立 —— 这正是原"宽度突变"判据把它误判成十字的根因。 */
#define CROSS_CORNER_JUMP    12    // 单侧边沿单行向外突变的最小列数（角点成立条件）
#define CROSS_CORNER_WINDOW  5     // 左下/右下角点允许相隔的最大行数
#define CROSS_TREND_ROWS     6     // 用角点下方多少行估计该侧边沿自身的趋势
#define CROSS_TREND_FACTOR   2.5f  // 突变量必须达到趋势斜率的多少倍才算"折角"
#define CROSS_ARM_MIN        120   // 横臂段宽度下限（列）：远弯的白区不会这么宽
#define CROSS_ARM_ROWS       10    // 角点之上多少行内必须出现横臂
#define CROSS_EDGE_MARGIN    2     // 边界距画面左右边缘多少列以内算"贴边"
#define CROSS_SAFE_TOP       4     // 距赛道搜索终点多少行以内不判定（避免"赛道到头"截断）
#define CROSS_MIN_TOP        20    // 搜索终点低于此行时任何元素识别都不可信

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
#define RING_MIN_SIZE        8     // 靶环外接矩形的最小边长（远处靶环只有 8~10 px）
#define RING_MAX_SIZE        52    // 靶环外接矩形的最大边长
#define RING_MIN_FILL        0.15f // 暗像素占比下限（太低是噪点）
#define RING_MAX_FILL        0.50f // 暗像素占比上限（实心暗块远大于它）
#define RING_MIN_ASPECT      0.6f  // 外接矩形长宽比范围（圆环应接近正方）
#define RING_MAX_ASPECT      1.8f
#define RING_NEED_HOLE       1     // 是否要求暗线闭合成环（有内孔）
#define RING_HOLE_MIN        0.25  // 内孔面积 / 外接矩形面积 的下限

#define FIT_WINDOW           22    // 十字补线时，用拐点下方多少行做最小二乘拟合

/* 弯道中线：不再对中线做任何平滑 / 拟合。
   近处直道的中线由"左右边沿取中点"直接得到，本身就是准的，原样保留 —— 对它做
   平滑或拟合只会引入帧间抖动（实车表现就是抖）。弯道段（有一侧边沿已经跑出画面）
   则换一条新的曲线来算：拿可信的那一侧边沿，沿道路宽度方向平移半个赛道宽。
   为什么不能对弯道的中线点做平滑：那一侧边沿是被画面截断的，取中点得到的位置
   本身就是错的，对错的点做平滑只会得到一条更光滑的错线。 */
#define CURVE_SIDE_LOST       2     // 边沿距画面左右缘多少列以内算"跑出画面"
#define CURVE_MIN_LOST_ROWS   3     // 连续多少行丢线才认定进入弯道段
#define CURVE_WIDTH_ROWS     16     // 用弯道起点下方多少行估计"宽度 ~ 行号"
#define CURVE_WIDTH_MIN_RATIO 0.55  // 弯道段宽度不低于起点宽度的该倍数（防止外推发散）
#define CURVE_BLEND_ROWS      3     // 与直道中线衔接的过渡行数
#define CURVE_EXT_FIT_ROWS   12     // 沿可信边沿向弯道尽头延伸时所用的最近行数
#define CURVE_EXT_MAX        18     // 沿新曲线最多向弯道尽头延伸多少行

#define WIDTH_FIT_TRIM       15    // 赛道宽度趋势拟合的残差阈值（列）
#define WIDTH_MIN_VALID      12    // 参与宽度趋势拟合的最小行宽

/* ---------------- 逐行扫描的中间结果 ----------------
   提到文件作用域，是为了让 detectTargetRing() 等元素识别函数直接复用，
   不必把整张扫描结果当参数层层传递。 */
static int g_row_ok[MAX_VIDEO_LINE];  // 本行赛道包络是否有效
static int g_row_l[MAX_VIDEO_LINE];   // 本行赛道左边界（列）
static int g_row_r[MAX_VIDEO_LINE];   // 本行赛道右边界（列）
static int g_row_frag[MAX_VIDEO_LINE];// 本行被斑马线黑条"切碎"后剩下的白块段数（0 = 未被切碎）
static int g_top_line = -1;           // 成功搜索到的最高行号
static int g_cross_l_row = -1;        // 左下角点所在行（-1 表示未找到）
static int g_cross_r_row = -1;        // 右下角点所在行（-1 表示未找到）
static int g_cross_l_far_row = -1;    // 左上角点所在行（横臂上沿回折点，可能被画面截断）
static int g_cross_r_far_row = -1;    // 右上角点所在行

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
 * @brief 判断某侧边沿在该行的跳变是不是一个"折角"。
 * @note  弯道处边沿也会逐行移动，但它是连续平滑的（每行 1~2 列）；
 *        十字横臂的折角是"一步到位"的（十几到几十列）。这里用角点下方
 *        CROSS_TREND_ROWS 行估出该侧自身的平均漂移量，跳变必须明显超过它。
 * @param edge 边沿列坐标数组（按行号索引）。
 * @param row 发生跳变的那一行。
 * @param jump 该行的跳变量（正值表示向外跳）。
 * @retval 1 是折角；0 不是。
 */
static int is_corner_jump(const int *edge, int row, int jump)
{
    const int span = MIN(row - 1, CROSS_TREND_ROWS);
    if(span < 2)
    {
        return 1; // 数据太少，不做趋势判断
    }
    const int trend = ABS_OF(edge[row - 1] - edge[row - 1 - span]) / span;
    return (jump >= (int)((float)trend * CROSS_TREND_FACTOR)) ? 1 : 0;
}

/**
 * @brief 找横臂上沿"回折"的角点（左上 / 右上角点）。
 * @note  横臂再往上，边界会折回赛道内侧，这个回折点就是左上/右上角点。
 *        正入十字常常只露出半个横臂（上沿被画面上边缘截断），所以这两个角点
 *        只作为加分确认，找不到不否决前面的判据。
 * @param edge 边沿列坐标数组。
 * @param from_row 从这一行往上找。
 * @param to_row 找到这一行为止。
 * @param inward_sign +1：左边界（向内 = 列号变大）；-1：右边界（向内 = 列号变小）。
 * @retval 回折点所在行，找不到返回 -1。
 */
static int find_return_corner(const int *edge, int from_row, int to_row, int inward_sign)
{
    for(int r = from_row + 1; r <= to_row; r++)
    {
        const int delta = (edge[r] - edge[r - 1]) * inward_sign;
        if(delta >= CROSS_CORNER_JUMP)
        {
            return r;
        }
    }
    return -1;
}

/**
 * @brief 四角点法识别"正入十字"。
 * @note  判据（三个条件同时满足）：
 *          1) 左下角点与右下角点在同一行附近（相差 <= CROSS_CORNER_WINDOW 行）同时出现
 *             —— 横向白臂是"左右一起甩出去"的，这是与远弯的本质区别
 *                （远弯只有一侧向外扩，另一侧是连续平滑的弯曲）；
 *          2) 角点之上出现"几乎横跨整幅画面"的宽白区（横臂，宽度 >= CROSS_ARM_MIN）；
 *          3) 横臂段至少一侧边界顶到画面边缘（横臂一直伸出画面外）。
 *        另外再尽力找左上/右上角点，作为横臂下沿~上沿的额外确认。
 *        全程只使用"本行与上一行有重叠的白块"，不再向上跨过黑色区域去找别的白块，
 *        保证赛道连通性 —— 不连通的白块绝不会被当成赛道。
 */
static void detectCrossByCorners(int top_line)
{
    g_cross_l_row = -1;
    g_cross_r_row = -1;
    g_cross_l_far_row = -1;
    g_cross_r_far_row = -1;
    cross_row = -1;

    if(top_line < CROSS_MIN_TOP)
    {
        return;
    }

    int l_cand[IMG_H];
    int r_cand[IMG_H];
    int l_num = 0;
    int r_num = 0;

    for(int r = 2; r <= top_line; r++)
    {
        if(!g_row_ok[r] || !g_row_ok[r - 1])
        {
            continue;
        }
        const int jump_l = g_row_l[r - 1] - g_row_l[r]; // >0：左边界向外（向左）折出角
        const int jump_r = g_row_r[r] - g_row_r[r - 1]; // >0：右边界向外（向右）折出角
        if(jump_l >= CROSS_CORNER_JUMP && is_corner_jump(g_row_l, r, jump_l))
        {
            l_cand[l_num++] = r;
        }
        if(jump_r >= CROSS_CORNER_JUMP && is_corner_jump(g_row_r, r, jump_r))
        {
            r_cand[r_num++] = r;
        }
    }

    int best_arm = 0;
    for(int i = 0; i < l_num; i++)
    {
        for(int j = 0; j < r_num; j++)
        {
            const int rl = l_cand[i];
            const int rr = r_cand[j];
            if(ABS_OF(rl - rr) > CROSS_CORNER_WINDOW)
            {
                continue; // 左下、右下角点必须"同时"出现
            }
            if(top_line < 20)
            {
                continue;
            }
            const int top = MAX(rl, rr);
            if(top > top_line - CROSS_SAFE_TOP)
            {
                continue; // 太靠近搜索终点：那是"赛道到头"，不是十字
            }

            int arm = 0;
            int touch = 0;
            const int arm_end = MIN(top + CROSS_ARM_ROWS, top_line);
            for(int k = top; k <= arm_end; k++)
            {
                if(!g_row_ok[k])
                {
                    continue;
                }
                const int w = g_row_r[k] - g_row_l[k];
                if(w > arm)
                {
                    arm = w;
                }
                if(g_row_l[k] <= CROSS_EDGE_MARGIN || g_row_r[k] >= IMG_W - 1 - CROSS_EDGE_MARGIN)
                {
                    touch = 1;
                }
            }
            if(arm < CROSS_ARM_MIN || !touch)
            {
                continue; // 只有一侧甩出去的弯道会被这一条挡掉
            }
            if(arm > best_arm)
            {
                best_arm = arm;
                g_cross_l_row = rl;
                g_cross_r_row = rr;
                g_cross_l_far_row = find_return_corner(g_row_l, top, top_line, +1);
                g_cross_r_far_row = find_return_corner(g_row_r, top, top_line, -1);
            }
        }
    }

    if(best_arm > 0)
    {
        cross_row = MAX(g_cross_l_row, g_cross_r_row); // 拐点取两条边沿中较高的那个角点
    }
}

/**
 * @brief 估计"赛道宽度随行号变化"的经验模型（透视：越远越窄）。
 * @note  单边丢线时要用"该行本来应该有多宽"来反推丢失的那一侧。固定的中位宽度
 *        在近处（约 142 列）和远处（约 45 列）差了三倍，直接用中位数会把补出来的线
 *        拉得离谱 —— 这也是原来近弯处中线会跳到画面中心的原因之一。
 *        这里用最小二乘直线拟合"宽度 ~ 行号"，并迭代剔除残差过大的行。
 * @param top_line 搜索终点行。
 * @param cross_top 十字横臂区起点（<=0 表示没有十字）。横臂区的宽度不代表赛道。
 * @param out_a 输出斜率。
 * @param out_b 输出截距。
 */
static void estimateRoadWidth(int top_line, int cross_top, double *out_a, double *out_b)
{
    static double xs[IMG_H];
    static double ws[IMG_H];

    if(out_a == NULL || out_b == NULL)
    {
        return;
    }

    int n = 0;
    const int limit = (cross_top > 0) ? MIN(cross_top, top_line) : top_line;
    for(int r = 0; r <= limit && r < IMG_H && n < IMG_H; r++)
    {
        if(!g_row_ok[r] || g_row_frag[r])
        {
            continue;
        }
        const double w = (double)(g_row_r[r] - g_row_l[r]);
        if(w < (double)WIDTH_MIN_VALID)
        {
            continue;
        }
        xs[n] = (double)r;
        ws[n] = w;
        n++;
    }

    if(n < 3)
    {
        *out_a = 0.0;
        *out_b = g_row_ok[0] ? (double)(g_row_r[0] - g_row_l[0]) : 60.0;
        return;
    }

    double a = 0.0;
    double b = 0.0;
    for(int iter = 0; iter < 2; iter++)
    {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        int m = 0;
        for(int i = 0; i < n; i++)
        {
            if(iter > 0 && ABS_OF(ws[i] - (a * xs[i] + b)) > (double)WIDTH_FIT_TRIM)
            {
                continue;
            }
            sx += xs[i];
            sy += ws[i];
            sxx += xs[i] * xs[i];
            sxy += xs[i] * ws[i];
            m++;
        }
        if(m < 3)
        {
            break;
        }
        const double den = (double)m * sxx - sx * sx;
        if(den <= 1e-6)
        {
            break;
        }
        a = ((double)m * sxy - sx * sy) / den;
        b = (sy - a * sx) / (double)m;
    }
    *out_a = a;
    *out_b = b;
}

/**
 * @brief 解 3x3 线性方程组 M * x = rhs（高斯消元，带主元）。
 * @retval 1 成功；0 奇异（不可解）。
 */
static int solve3(double m[3][3], double *rhs)
{
    for(int col = 0; col < 3; col++)
    {
        int pivot = col;
        for(int r = col + 1; r < 3; r++)
        {
            if(ABS_OF(m[r][col]) > ABS_OF(m[pivot][col]))
            {
                pivot = r;
            }
        }
        if(ABS_OF(m[pivot][col]) < 1e-9)
        {
            return 0;
        }
        if(pivot != col)
        {
            for(int c = 0; c < 3; c++)
            {
                const double tmp = m[col][c];
                m[col][c] = m[pivot][c];
                m[pivot][c] = tmp;
            }
            const double tr = rhs[col];
            rhs[col] = rhs[pivot];
            rhs[pivot] = tr;
        }
        for(int r = col + 1; r < 3; r++)
        {
            const double factor = m[r][col] / m[col][col];
            for(int c = col; c < 3; c++)
            {
                m[r][c] -= factor * m[col][c];
            }
            rhs[r] -= factor * rhs[col];
        }
    }
    for(int r = 2; r >= 0; r--)
    {
        double sum = rhs[r];
        for(int c = r + 1; c < 3; c++)
        {
            sum -= m[r][c] * rhs[c];
        }
        rhs[r] = sum / m[r][r];
    }
    return 1;
}

/**
 * @brief 估计"赛道宽度随行号"的局部趋势（只服务于弯道段的单侧补宽）。
 * @note  一侧边沿跑出画面时，必须知道"这一行本来该有多宽"。全图中位宽度在近处
 *        约 142 列、远处约 45 列，差了三倍；而全局趋势线在弯道起点附近也会差出
 *        十几列（弯道处道路在画面里是斜的，水平切出来的宽度和直线段不是一回事）。
 *        这里只看弯道起点下方最近 CURVE_WIDTH_ROWS 行，得到"刚要进弯时"的真实
 *        宽度与变化率，再顺着它往远处推，既贴合透视又不会发散。
 * @param end_idx   只使用下标 < end_idx 的行（即弯道起点及其以下的行）。
 * @param out_w0    输出弯道起点处的赛道宽度。
 * @param out_slope 输出每前进一行宽度的变化量。
 */
static void estimateLocalWidth(int end_idx, double *out_w0, double *out_slope)
{
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    int n = 0;

    int begin = end_idx - CURVE_WIDTH_ROWS;
    if(begin < 0)
    {
        begin = 0;
    }

    for(int i = begin; i < end_idx && i < edge_num_l && i < edge_num_r; i++)
    {
        const int w = (int)edge_r[i].y - (int)edge_l[i].y;
        if(w < WIDTH_MIN_VALID)
        {
            continue;
        }
        const double x = (double)edge_l[i].x;
        const double y = (double)w;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        n++;
    }

    const double x_ref = (end_idx > 0 && end_idx <= edge_num_l)
                             ? (double)edge_l[end_idx - 1].x
                             : 0.0;

    if(n < 3)
    {
        *out_w0 = (double)g_road_width;
        *out_slope = 0.0;
        return;
    }

    const double den = (double)n * sxx - sx * sx;
    if(den <= 1e-6)
    {
        *out_w0 = sy / (double)n;
        *out_slope = 0.0;
        return;
    }

    const double a = ((double)n * sxy - sx * sy) / den;
    const double b = (sy - a * sx) / (double)n;
    *out_slope = a;
    *out_w0 = a * x_ref + b;
}

/**
 * @brief 靶环识别（在灰度图上做，OTSU 二值图上靶环基本不可见）。
 * @note  靶环的真实形态是"一小片亮底靶纸 + 上面一圈很细的暗圆环"。判据分三层：
 *          1) 在赛道内部（左右边沿各内缩 4 列以避开边界过渡带）统计灰度均值；
 *          2) 取低于均值 RING_DARK_OFFSET 的暗像素，再闭运算把断成几段的圆环连起来；
 *          3) 找最外层轮廓：外接矩形接近正方、尺寸适中、"暗像素 / 外接矩形" 很小
 *             （细线特征），并且暗线闭合成环（有子轮廓 = 有内孔）；
 *          4) 内孔面积 / 外接矩形面积 必须够大（≥ RING_HOLE_MIN）。第 3 条在赛道
 *             边界、十字拐角、弯道内侧的暗区上也会成立，实测产生大量误报；这些"假环"
 *             的内孔都很小，用第 4 条可以全部滤掉，且与真靶环之间有 8 倍以上差距。
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

        /* 内孔必须足够大（真圆环的最强特征）。
           靶环的暗线只有 1~2 px 粗，内孔面积约占外接矩形的一半；而赛道边界、十字
           拐角、弯道内侧的暗区经闭运算后虽然也能围出一个"内孔"，但那个孔很小。
           实测本序列全部候选的"内孔面积 / 外接矩形面积"呈两极分布：
             真靶环 0.448 / 0.453 / 0.460 / 0.516
             各类误报 0.006 / 0.009 / 0.023 / 0.033 / 0.036 / 0.050 / 0.051 / 0.052
           取 0.25 作阈值，两侧都有 4 倍以上余量，误报可以全部滤掉。 */
        const double hole_area = cv::contourArea(contours[hierarchy[i][2]]);
        if(hole_area / box_area < RING_HOLE_MIN)
        {
            continue;
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
 *          1) 自下而上逐行提取白块，只保留"与上一行赛道区间有重叠"的白块来连通赛道
 *             —— 连通性是搜赛道的必要条件，跨过黑色区域去找别的白块会把中线
 *             拉到不连通的另一块赛道上，直接导致出界；
 *          2) 记录内部黑洞（用于区分斑马线 / 障碍块）；
 *          3) 正入十字：四角点法判定（左下/右下角点同时出现 + 横臂够宽 + 贴边），
 *             拐点以上用最小二乘直线外推补线；
 *          4) 弯道中线：单边丢线用另一侧 + 该行应有宽度补齐 -> 野点剔除 -> 滑动平均
 *             -> 二次曲线鲁棒拟合 -> 沿曲线延伸到弯道尽头。
 */
void Search()
{
    variable_init();

    static int width_hist[IMG_H];          // 各行赛道宽度，用于取中位数

    // 赛道包络结果直接落在文件作用域数组上，元素识别函数（如 detectTargetRing）要复用
    int *row_l = g_row_l;
    int *row_r = g_row_r;
    int *row_ok = g_row_ok;
    int *row_frag = g_row_frag;

    int line = 0;       // 当前搜索行
    int top_line = -1;  // 成功搜到的最高行
    int width_cnt = 0;  // width_hist 的有效长度

    // 逐行向上搜索
    for(line = 0; line < IMG_H; line++)
    {
        row_l[line] = 0;
        row_r[line] = 0;
        row_ok[line] = 0;
        row_frag[line] = 0;

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


        // 单独处理第一行，这一行选择的白块是后续找连通白块的基础
        if(line == 0)
        {
            if(white_block_num == 0)
            {
                return; // 第一行没有白块，判定为出界
            }

            /* 斑马线压到画面最底端时，这一行会被黑条切成一大堆小碎块。
               原来直接取"最宽的单块"，实测左边界会偏掉 80 多列（真值约 14 列，
               却取到 98 列），而且之后要好几行才能把边界恢复回来，中线因此断裂。
               这里先把间隙很小的碎块合并成组，再取最宽的一组作为赛道。 */
            int group_start[MAX_WB_NUM];
            int group_end[MAX_WB_NUM];
            int group_num = 0;
            for(int i = 0; i < white_block_num; i++)
            {
                if(group_num > 0 && (wb_start[i] - group_end[group_num - 1] - 1) <= ZEBRA_MERGE_GAP)
                {
                    group_end[group_num - 1] = wb_end[i];
                }
                else
                {
                    group_start[group_num] = wb_start[i];
                    group_end[group_num] = wb_end[i];
                    group_num++;
                }
            }

            int base_group = 0;
            int group_width = group_end[0] - group_start[0];
            for(int i = 1; i < group_num; i++)
            {
                const int temp = group_end[i] - group_start[i];
                if(temp > group_width)
                {
                    group_width = temp;
                    base_group = i;
                }
            }
            row_l[0] = group_start[base_group];
            row_r[0] = group_end[base_group];
            row_ok[0] = 1;

            // 首行是否被"切碎"（用于斑马线识别）
            {
                int frag = 0;
                if(white_block_num >= ZEBRA_MIN_BLOCKS)
                {
                    int gap_max = 0;
                    for(int i = 1; i < white_block_num; i++)
                    {
                        const int gap = wb_start[i] - wb_end[i - 1] - 1;
                        if(gap > gap_max)
                        {
                            gap_max = gap;
                        }
                    }
                    frag = (gap_max > 0 && gap_max <= ZEBRA_MERGE_GAP) ? white_block_num : 0;
                }
                row_frag[0] = frag;
            }

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
        for(int k = 1; k < cover_num; k++)
        {
            const int a = cover_idx[k - 1];
            const int b = cover_idx[k];
            const int gap = wb_start[b] - wb_end[a] - 1;
            if(gap > gap_max)
            {
                gap_max = gap;
            }
        }

        /* ---------- 3. 本行被切碎的程度（斑马线候选） ----------
           斑马线是赛道上的黑色细横条，会把某一行切成很多小碎块，段间间隙很小
           而赛道整体宽度基本不变。这里把"本行的白块段数"记下来（0 = 没被切碎），
           具体判定放在第 7 步统一处理。 */
        if(cover_num >= ZEBRA_MIN_BLOCKS && gap_max > 0 && gap_max <= ZEBRA_MERGE_GAP)
        {
            row_frag[line] = cover_num;
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

    /* ---------- 4. 正入十字识别（四角点法） ----------
       注意：这里已经删掉了原来"搜线结束之后再向上检查是否还有横跨图像中部的白块"
       的补判逻辑。那段逻辑会跨过黑色区域去认另一块赛道，破坏"赛道必须连通"这个
       前提 —— 实际跑车中画面里完全可能出现另一块与本赛道不连通的赛道，那样会把
       中线直接拉到别的赛道上去，导致出界。元素识别一律只在已经连通的赛道包络里做。 */
    detectCrossByCorners(top_line);

    /* ---------- 5. 十字区域边界补线 ----------
       拐点以上的边沿来自横向白臂，不能再用；用该侧拐点下方的有效边沿做最小二乘拟合，
       再按该直线把这一侧的边沿一路补齐到图像顶部，使十字路口内中线保持平直（直行通过）。
       左右两侧的角点可能不在同一行，所以两侧分别从各自的角点开始外推。 */
    if(cross_row > 3)
    {
        extrapolate_edge(edge_l, &edge_num_l, MAX(3, g_cross_l_row), IMG_H - 1);
        extrapolate_edge(edge_r, &edge_num_r, MAX(3, g_cross_r_row), IMG_H - 1);
    }

    /* ---------- 6. 估计赛道宽度 ----------
       除了保留原来的中位宽度，还要估一条"宽度 ~ 行号"的趋势线：
       近处赛道约 142 列、远处约 45 列，单边丢线时必须按该行应有的宽度来反推。 */
    double width_a = 0.0;
    double width_b = 0.0;
    {
        const int width_ref = median_of(width_hist, width_cnt);
        if(width_ref > 8)
        {
            g_road_width = width_ref;
        }
        estimateRoadWidth(top_line, cross_row, &width_a, &width_b);
    }

    /* ---------- 7. 斑马线识别 ----------
       斑马线是赛道上的黑色细横条，会把某一行切成很多小段白块（段间间隙很小），
       而赛道整体宽度不变，因此"连续多行被切碎 + 宽度仍然正常"即为斑马线。
       宽度用该行的"应有宽度"来比，比用全图中位宽度更稳（近处 142 列、远处 45 列）。

       为什么还要加"峰值碎块数"这一条：只要求"连续 3 行、每行 ≥3 段"时，赛道上很多
       东西都能满足 —— 大灯眩光、靶环的靶纸方块、十字附近的暗结构，实测在 830 / 840
       帧以及靶环段 950-955 帧都会误报。真斑马线的黑条有 5~9 根，切出来的白块段数
       峰值能到 9；而上述误报的段数峰值只有 3~4。因此要求"这一串碎块行里最密的一行
       段数 ≥ ZEBRA_PEAK_BLOCKS"，既能滤掉误报，又不会因为斑马线远端几条黑条糊在一
       起（段数掉到 3）而漏掉整段。 */
    {
        int run_start = -1;
        int run_peak = 0;
        for(int x = 0; x <= IMG_H; x++)
        {
            int fragments = 0;
            if(x < IMG_H && row_ok[x] && row_frag[x] >= ZEBRA_MIN_BLOCKS)
            {
                double expect_w = width_a * (double)x + width_b;
                if(expect_w < (double)WIDTH_MIN_VALID)
                {
                    expect_w = (double)WIDTH_MIN_VALID;
                }
                if((double)(row_r[x] - row_l[x]) <= expect_w * ZEBRA_WIDTH_MAX)
                {
                    fragments = row_frag[x];
                }
            }

            if(fragments > 0)
            {
                if(run_start < 0)
                {
                    run_start = x;
                    run_peak = fragments;
                }
                else if(fragments > run_peak)
                {
                    run_peak = fragments;
                }
            }
            else if(run_start >= 0)
            {
                if((x - run_start) >= ZEBRA_MIN_ROWS && run_peak >= ZEBRA_PEAK_BLOCKS)
                {
                    zebra_row_start = run_start;
                    zebra_row_end = x - 1;
                }
                run_start = -1;
                run_peak = 0;
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

    /* ---------- 9. 中线构造 ----------
       分两种情形，各自就地取用、互不干扰：

       (a) 近处直道 —— 左右两条边沿都看得见，"取中点"得到的中线本身就是准的。
           原样使用，不做任何平滑、拟合、滑动平均：任何对它的加工都只会引入
           帧间抖动（实车表现就是方向抖）。

       (b) 弯道段 —— 有一侧边沿已经跑出画面，被画面边缘截断，此时"取中点"得到的
           是"一条真边沿 + 一条假边沿"的中点，本身就是错的；对错的点再平滑，得到的
           仍然是一条错线。这里改用一个新办法算中线：取可信的那一侧边沿（真实边界），
           沿道路宽度方向平移半个赛道宽，得到一条弯曲、连续、且始终落在赛道中间的新
           曲线。可信边沿本身是准的，所以这条新曲线是准的。

       全程没有对中线点做最小二乘或平滑。 */
    mid_point_num = edge_num_l;
    if(mid_point_num > edge_num_r)
    {
        mid_point_num = edge_num_r;
    }
    if(mid_point_num > MAX_MID_POINT_NUM)
    {
        mid_point_num = MAX_MID_POINT_NUM;
    }

    // 十字横臂区由第 5 步的外推负责，不做弯道处理
    const int cross_zone = (cross_row > 0) ? MIN(g_cross_l_row, g_cross_r_row) : IMG_H;

    // 9.1 基准：逐行取中点。直道段这就是最终结果，下面不会再改动它。
    for(int i = 0; i < mid_point_num; i++)
    {
        mid_points[i].x = edge_l[i].x;
        mid_points[i].y = (int16_t)(((int)edge_l[i].y + (int)edge_r[i].y) / 2);
    }

    // 9.2 找弯道起点：第一处"连续 CURVE_MIN_LOST_ROWS 行有一侧边沿贴到画面缘"
    int bend_idx = -1;   // 弯道起点在中线点数组中的下标
    int bend_side = 0;   // +1：右边沿丢，用左边沿向右平移；-1：左边沿丢，用右边沿向左平移
    for(int i = 0; i + CURVE_MIN_LOST_ROWS <= mid_point_num; i++)
    {
        if(edge_l[i].x >= cross_zone)
        {
            break;
        }
        int lost_l = 1;
        int lost_r = 1;
        for(int k = 0; k < CURVE_MIN_LOST_ROWS; k++)
        {
            if(edge_l[i + k].y > CURVE_SIDE_LOST)
            {
                lost_l = 0;
            }
            if(edge_r[i + k].y < IMG_W - 1 - CURVE_SIDE_LOST)
            {
                lost_r = 0;
            }
        }
        if(lost_l && lost_r)
        {
            bend_idx = i;
            bend_side = 0;   // 两侧都被截断，没有可信边沿可用，维持中点
            break;
        }
        if(lost_l || lost_r)
        {
            bend_idx = i;
            bend_side = lost_r ? 1 : -1;
            break;
        }
    }

    // 9.3 弯道段：用可信边沿 + 半个赛道宽，另算一条新曲线
    if(bend_idx >= 0 && bend_side != 0)
    {
        double w0 = 0.0;      // 弯道起点处的赛道宽度
        double w_slope = 0.0; // 每前进一行宽度的变化量
        estimateLocalWidth(bend_idx, &w0, &w_slope);
        if(w0 < (double)WIDTH_MIN_VALID)
        {
            w0 = (double)WIDTH_MIN_VALID;
        }

        const int bend_row = edge_l[bend_idx].x;
        int join_init = 0;
        double join_delta = 0.0; // 与直道末行之间的台阶，在开头 CURVE_BLEND_ROWS 行内衰减掉

        for(int i = bend_idx; i < mid_point_num; i++)
        {
            const int row = edge_l[i].x;
            if(row >= cross_zone)
            {
                break;
            }

            const int L = edge_l[i].y;
            const int R = edge_r[i].y;
            const int lost_l = (L <= CURVE_SIDE_LOST);
            const int lost_r = (R >= IMG_W - 1 - CURVE_SIDE_LOST);

            if(lost_l && lost_r)
            {
                continue; // 两侧都不可信，维持中点
            }
            if(!lost_l && !lost_r)
            {
                continue; // 两侧都看得见，中点就是准的，保持不动
            }

            double w = w0 + w_slope * (double)(row - bend_row);
            if(w < w0 * CURVE_WIDTH_MIN_RATIO)
            {
                w = w0 * CURVE_WIDTH_MIN_RATIO; // 防止趋势外推发散
            }
            if(w > (double)IMG_W)
            {
                w = (double)IMG_W;
            }

            // 右边沿丢 -> 沿左边沿向右平移半个赛道宽；左边沿丢 -> 沿右边沿向左平移
            double mid = (bend_side > 0) ? ((double)L + w * 0.5)
                                         : ((double)R - w * 0.5);

            if(!join_init && bend_idx > 0)
            {
                join_delta = (double)mid_points[bend_idx - 1].y - mid;
                join_init = 1;
            }
            const int k = i - bend_idx;
            if(k < CURVE_BLEND_ROWS)
            {
                mid += join_delta * (double)(CURVE_BLEND_ROWS - k) / (double)CURVE_BLEND_ROWS;
            }

            if(mid < 0.0)
            {
                mid = 0.0;
            }
            if(mid > (double)(IMG_W - 1))
            {
                mid = (double)(IMG_W - 1);
            }
            mid_points[i].y = (int16_t)(mid + 0.5);
        }

        /* 9.4 沿新曲线向弯道尽头延伸。
           弯道尽头很多时候已经出了画面或者搜线提前终止，但中线需要继续画过去。
           做法：把"可信边沿"最近的 CURVE_EXT_FIT_ROWS 行做一次二次拟合，用这条
           边沿曲线往外推，中线仍旧取"边沿曲线 ± 半个赛道宽"。
           注意拟合对象是真实、准确的边沿，不是那些不准的中线点。 */
        if(bend_idx < mid_point_num && cross_row <= 3)
        {
            const int last_row = edge_l[mid_point_num - 1].x;
            int tail = mid_point_num - bend_idx;
            if(tail > CURVE_EXT_FIT_ROWS)
            {
                tail = CURVE_EXT_FIT_ROWS;
            }

            if(tail >= 4 && last_row < IMG_H - 1 && mid_point_num < MAX_MID_POINT_NUM)
            {
                double ubar = 0.0;
                for(int i = mid_point_num - tail; i < mid_point_num; i++)
                {
                    ubar += (double)edge_l[i].x;
                }
                ubar /= (double)tail;

                double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0, d4 = 0.0;
                double t0 = 0.0, t1 = 0.0, t2 = 0.0;
                for(int i = mid_point_num - tail; i < mid_point_num; i++)
                {
                    const double u = (double)edge_l[i].x - ubar;
                    const double v = (bend_side > 0) ? (double)edge_l[i].y : (double)edge_r[i].y;
                    const double u2 = u * u;
                    d0 += 1.0;
                    d1 += u;
                    d2 += u2;
                    d3 += u2 * u;
                    d4 += u2 * u2;
                    t0 += v;
                    t1 += u * v;
                    t2 += u2 * v;
                }

                double mat[3][3] = {{d4, d3, d2}, {d3, d2, d1}, {d2, d1, d0}};
                double rhs[3] = {t2, t1, t0};
                if(solve3(mat, rhs))
                {
                    const double qa = rhs[0];
                    const double qb = rhs[1];
                    const double qc = rhs[2];

                    int row = last_row;
                    int added = 0;
                    int ext_init = 0;
                    double ext_delta = 0.0;
                    while(row + 1 < IMG_H && added < CURVE_EXT_MAX && mid_point_num < MAX_MID_POINT_NUM)
                    {
                        row++;
                        const double u = (double)row - ubar;
                        const double e = qa * u * u + qb * u + qc;

                        double w = w0 + w_slope * (double)(row - bend_row);
                        if(w < w0 * CURVE_WIDTH_MIN_RATIO)
                        {
                            w = w0 * CURVE_WIDTH_MIN_RATIO;
                        }
                        double mid = (bend_side > 0) ? (e + w * 0.5) : (e - w * 0.5);
                        if(mid < 0.0 || mid > (double)(IMG_W - 1))
                        {
                            break; // 新曲线已经走出画面，弯道到头
                        }

                        if(!ext_init)
                        {
                            ext_delta = (double)mid_points[mid_point_num - 1].y - mid;
                            ext_init = 1;
                        }
                        if(added < CURVE_BLEND_ROWS)
                        {
                            mid += ext_delta * (double)(CURVE_BLEND_ROWS - added) / (double)CURVE_BLEND_ROWS;
                        }

                        if(mid < 0.0)
                        {
                            mid = 0.0;
                        }
                        if(mid > (double)(IMG_W - 1))
                        {
                            mid = (double)(IMG_W - 1);
                        }

                        mid_points[mid_point_num].x = (int16_t)row;
                        mid_points[mid_point_num].y = (int16_t)(mid + 0.5);
                        mid_point_num++;
                        added++;
                    }
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

    // 绘制中线（搜索终点以上是沿拟合曲线"延伸到弯道尽头"的段，用橙色区分）
    for(int i = 0; i < mid_point_num; i++)
    {
        const int px = mid_points[i].y;
        const int py = MAX_VIDEO_LINE - 1 - mid_points[i].x;

        if(px >= 0 && px < display.cols && py >= 0 && py < display.rows)
        {
            const bool is_ext = (g_top_line >= 0 && mid_points[i].x > g_top_line);
            cv::circle(display, cv::Point(px, py), 1, is_ext ? orange : blue, -1);
        }
    }

    // 十字四角点：左下/右下角点用实心圆，左上/右上角点用空心圆，方便在视频里核对判据
    if(g_cross_l_row > 0 && g_row_ok[g_cross_l_row - 1])
    {
        cv::circle(display, cv::Point(g_row_l[g_cross_l_row - 1], MAX_VIDEO_LINE - 1 - (g_cross_l_row - 1)),
                   2, cyan, -1, cv::LINE_AA);
    }
    if(g_cross_r_row > 0 && g_row_ok[g_cross_r_row - 1])
    {
        cv::circle(display, cv::Point(g_row_r[g_cross_r_row - 1], MAX_VIDEO_LINE - 1 - (g_cross_r_row - 1)),
                   2, cyan, -1, cv::LINE_AA);
    }
    if(g_cross_l_far_row > 0 && g_row_ok[g_cross_l_far_row - 1])
    {
        cv::circle(display, cv::Point(g_row_l[g_cross_l_far_row - 1], MAX_VIDEO_LINE - 1 - (g_cross_l_far_row - 1)),
                   2, cyan, 1, cv::LINE_AA);
    }
    if(g_cross_r_far_row > 0 && g_row_ok[g_cross_r_far_row - 1])
    {
        cv::circle(display, cv::Point(g_row_r[g_cross_r_far_row - 1], MAX_VIDEO_LINE - 1 - (g_cross_r_far_row - 1)),
                   2, cyan, 1, cv::LINE_AA);
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
