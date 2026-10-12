/*
 * 临时探针 9：逐帧打印转向中心，用于衡量"实车会不会抖"。
 *
 * 车辆最终只用 turning_center 一个值来打角，所以"相邻帧转向中心的变化量"
 * 才是真正对应实车抖动的指标 —— 中线逐行抖动只是中间量。
 *
 * 输出：TC <帧号> <转向中心> <中点数>
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
        std::printf("TC %d %d %d\n", image_process_start_idx, turning_center, mid_point_num);
        std::fflush(stdout);
    }
    return 0;
}
