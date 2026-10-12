/*
 * 临时探针 6：衡量本轮弯道改动的三条验收指标。
 *
 * 1) 直道抖动  JIT  —— 取转向控制用到的行（0..TURNING_CALC_HEIGHT），
 *                     统计相邻帧同一行上中线列的变化量（max / mean）。
 *                     直道的中线本来就准，任何加工都只会让这个数变大。
 * 2) 中线出界  BAD  —— 某一行有一侧边沿跑出画面时，可信侧边沿记为 E，
 *                     中线必须落在赛道内（换句话说必须排在 E 的正确一侧）。
 *                     统计"中线跑到了可信边沿外侧"的行数。
 * 3) 弯道延伸  EXT  —— mid_point_num 比搜索终点 top+1 多出来的行数，
 *                     即沿新曲线补到弯道尽头的长度。
 *
 * 另打印每帧结尾 8 个中线点（EXT 段的形状），便于肉眼核对。
 *
 * 用法：./project            （扫描全序列）
 */
#include "camera.h"
#include "image.h"

#include <cstdio>

static int prev_mid[MAX_VIDEO_LINE];
static int have_prev = 0;

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

        int cur[MAX_VIDEO_LINE];
        for(int r = 0; r < MAX_VIDEO_LINE; r++)
        {
            cur[r] = -10000;
        }
        for(int i = 0; i < mid_point_num; i++)
        {
            const int r = mid_points[i].x;
            if(r >= 0 && r < MAX_VIDEO_LINE)
            {
                cur[r] = mid_points[i].y;
            }
        }

        // 1) 直道段帧间抖动
        int max_jit = 0;
        int sum_jit = 0;
        int cnt_jit = 0;
        if(have_prev)
        {
            for(int r = 0; r <= TURNING_CALC_HEIGHT; r++)
            {
                if(cur[r] == -10000 || prev_mid[r] == -10000)
                {
                    continue;
                }
                int d = cur[r] - prev_mid[r];
                if(d < 0)
                {
                    d = -d;
                }
                sum_jit += d;
                cnt_jit++;
                if(d > max_jit)
                {
                    max_jit = d;
                }
            }
        }

        // 2) 中线跑到可信边沿外侧的行数
        int bad_rows = 0;
        int bend_rows = 0;
        for(int i = 0; i < mid_point_num && i < edge_num_l && i < edge_num_r; i++)
        {
            const int row = edge_l[i].x;
            const int L = edge_l[i].y;
            const int R = edge_r[i].y;
            const int lost_l = (L <= 1);
            const int lost_r = (R >= MAX_VIDEO_POINT - 2);
            if(!(lost_l || lost_r))
            {
                continue;
            }
            bend_rows++;
            const int m = cur[row];
            if(m == -10000)
            {
                continue;
            }
            if(lost_r && !lost_l && m < L)
            {
                bad_rows++; // 左边沿可信，中线却排到了左边沿左侧
            }
            if(lost_l && !lost_r && m > R)
            {
                bad_rows++; // 右边沿可信，中线却排到了右边沿右侧
            }
        }

        // 3) 弯道延伸行数
        int top = 0;
        for(int i = 0; i < edge_num_l; i++)
        {
            if(edge_l[i].x > top)
            {
                top = edge_l[i].x;
            }
        }
        const int ext_rows = (mid_point_num > top + 1) ? (mid_point_num - (top + 1)) : 0;

        std::printf("JIT %d maxjit=%d meanjit=%d cnt=%d bad=%d bendrows=%d ext=%d midnum=%d top=%d\n",
                    image_process_start_idx, max_jit,
                    (cnt_jit > 0) ? (sum_jit / cnt_jit) : 0, cnt_jit,
                    bad_rows, bend_rows, ext_rows, mid_point_num, top);

        if(ext_rows > 0 && (image_process_start_idx % 37) == 0)
        {
            std::printf("  TAIL:");
            for(int i = mid_point_num - 8; i < mid_point_num; i++)
            {
                if(i >= 0)
                {
                    std::printf(" (%d,%d)", (int)mid_points[i].x, (int)mid_points[i].y);
                }
            }
            std::printf("\n");
        }

        for(int r = 0; r < MAX_VIDEO_LINE; r++)
        {
            prev_mid[r] = cur[r];
        }
        have_prev = 1;
        std::fflush(stdout);
    }
    return 0;
}
