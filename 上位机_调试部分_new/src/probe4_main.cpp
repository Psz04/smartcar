/*
 * 临时探针 4：逐行 dump 指定帧的左右边沿与中线，用于核对弯道段的边沿/中线形态。
 *
 * 用法：./project 40 60 200 ...
 *   每帧打印：
 *     FRAME <idx> top=<搜索终点行> midnum=<中线点数> tc=<转向中心> cross=<十字行>
 *                 zebra=<起>,<止> obs=<障碍> ring=<靶环>
 *     R <行号> <左边界> <右边界> <LOSTL 左边界贴到画面左缘> <LOSTR 右边界贴到画面右缘> <中线列>
 */
#include "camera.h"
#include "image.h"

#include <cstdio>
#include <cstdlib>

static int want[128];
static int want_n = 0;

static int is_want(int idx)
{
    for(int i = 0; i < want_n; i++)
    {
        if(want[i] == idx)
        {
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    for(int i = 1; i < argc && want_n < 128; i++)
    {
        want[want_n++] = atoi(argv[i]);
    }

    for(; image_process_start_idx <= image_process_end_idx; ++image_process_start_idx)
    {
        if(!ReadImage())
        {
            continue;
        }
        Search();
        calcTurningCenter();

        if(!is_want(image_process_start_idx))
        {
            continue;
        }

        int top = 0;
        for(int i = 0; i < edge_num_l; i++)
        {
            if(edge_l[i].x > top)
            {
                top = edge_l[i].x;
            }
        }

        std::printf("FRAME %d top=%d midnum=%d tc=%d cross=%d zebra=%d,%d obs=%d ring=%d\n",
                    image_process_start_idx, top, mid_point_num, turning_center,
                    cross_row, zebra_row_start, zebra_row_end, obstacle_found, ring_found);

        for(int i = 0; i < edge_num_l; i++)
        {
            const int row = edge_l[i].x;
            const int L = edge_l[i].y;
            const int R = (i < edge_num_r) ? edge_r[i].y : -1;
            const int mid = (i < mid_point_num) ? (int)mid_points[i].y : -1;
            std::printf("R %3d L=%3d R=%3d%s%s mid=%3d  w=%3d\n",
                        row, L, R,
                        (L <= 1) ? " LOSTL" : "",
                        (R >= MAX_VIDEO_POINT - 2) ? " LOSTR" : "",
                        mid, R - L);
        }
        std::printf("\n");
    }
    return 0;
}
