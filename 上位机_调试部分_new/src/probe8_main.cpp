/*
 * 临时探针 8：整条中线数组 + 逐行"边沿是否贴画面缘"标记的原始转储。
 *
 * 输出格式（每帧一行）：
 *   D <帧号> <中点数> <搜索终点top> <基准行数base> <y0> <y1> ... <y_base-1> <pin串>
 *   - y_k    ：第 k 个中线点的列坐标（数组下标即行号）
 *   - pin串  ：长度 base 的字符串，第 k 个字符描述第 k 行边沿状态
 *              '.' 两侧都在画面内（直道，中线就是取中点）
 *              'L' 左边沿贴左画面缘   'R' 右边沿贴右画面缘   'B' 两侧都贴
 *
 * 判据：本版相对"只取中点"基线的每一处差异，其所在行的 pin 必须是 'L' 或 'R'。
 *       若出现某行 pin=='.' 却与基线不同，即说明直道被加工过 —— 违例。
 *
 * 用法：./project
 */
#include "camera.h"
#include "image.h"

#include <cstdio>

int main()
{
    for(; image_process_start_idx <= image_process_end_idx; ++image_process_start_idx)
    {
        if(!ReadImage())
        {
            continue;
        }
        Search();
        calcTurningCenter();

        int top = 0;
        for(int i = 0; i < edge_num_l; i++)
        {
            if(edge_l[i].x > top)
            {
                top = edge_l[i].x;
            }
        }

        int base = (edge_num_l < edge_num_r) ? edge_num_l : edge_num_r;
        if(base > MAX_MID_POINT_NUM)
        {
            base = MAX_MID_POINT_NUM;
        }

        std::printf("D %d %d %d %d", image_process_start_idx, mid_point_num, top, base);
        for(int i = 0; i < mid_point_num; i++)
        {
            std::printf(" %d", (int)mid_points[i].y);
        }
        std::printf(" ");
        for(int i = 0; i < base; i++)
        {
            const int L = (int)edge_l[i].y;
            const int R = (int)edge_r[i].y;
            const int pl = (L <= 2);
            const int pr = (R >= MAX_VIDEO_POINT - 3);
            std::printf("%c", (pl && pr) ? 'B' : (pl ? 'L' : (pr ? 'R' : '.')));
        }
        std::printf("\n");
        std::fflush(stdout);
    }
    return 0;
}
