/*
 * 临时探针 5：全序列扫描"边沿丢线"情况。
 * 对每一帧输出：
 *   SCAN <帧号> top=<终点行> midnum=<中线点数> lostL=<首个左边界贴画左缘的行,-1无> lostR=<首个右边界贴画右缘的行,-1无> cross=<十字行>
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

        int lostL = -1;
        int lostR = -1;
        for(int i = 0; i < edge_num_l; i++)
        {
            const int row = edge_l[i].x;
            const int L = edge_l[i].y;
            const int R = (i < edge_num_r) ? edge_r[i].y : -1;
            if(lostL < 0 && L <= 1)
            {
                lostL = row;
            }
            if(lostR < 0 && R >= MAX_VIDEO_POINT - 2)
            {
                lostR = row;
            }
        }

        std::printf("SCAN %d top=%d midnum=%d lostL=%d lostR=%d cross=%d\n",
                    image_process_start_idx, top, mid_point_num, lostL, lostR, cross_row);
        std::fflush(stdout);
    }
    return 0;
}
