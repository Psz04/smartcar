/*
 * 临时探针 7：标定"赛道宽度 ~ 行号"曲线。
 * 只统计两侧边沿都在画面内、且宽度合理的行，逐行取中位数（跨全序列），
 * 这样得到的宽度曲线只跟相机几何有关，不受单侧丢线帧的污染。
 */
#include "camera.h"
#include "image.h"

#include <algorithm>
#include <cstdio>
#include <vector>

static std::vector<int> g_w[MAX_VIDEO_LINE];

int main()
{
    for(; image_process_start_idx <= image_process_end_idx; ++image_process_start_idx)
    {
        if(!ReadImage())
        {
            continue;
        }
        Search();

        for(int i = 0; i < edge_num_l && i < edge_num_r; i++)
        {
            const int row = edge_l[i].x;
            const int L = edge_l[i].y;
            const int R = edge_r[i].y;
            if(L <= 1 || R >= MAX_VIDEO_POINT - 2)
            {
                continue; // 只统计两侧都有效的行
            }
            const int w = R - L;
            if(w < 20)
            {
                continue;
            }
            if(row >= 0 && row < MAX_VIDEO_LINE)
            {
                g_w[row].push_back(w);
            }
        }
    }

    for(int r = 0; r < MAX_VIDEO_LINE; r++)
    {
        if(g_w[r].empty())
        {
            std::printf("W %d n=0\n", r);
            continue;
        }
        std::sort(g_w[r].begin(), g_w[r].end());
        const size_t n = g_w[r].size();
        std::printf("W %d n=%zu med=%d p25=%d p75=%d\n",
                    r, n, g_w[r][n / 2], g_w[r][n / 4], g_w[r][3 * n / 4]);
    }
    return 0;
}
