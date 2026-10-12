#include "camera.h"
#include "image.h"

/*
使用方法：
调试图像代码：
1. 把要调试的图像放到 input 文件夹内
2. 调整 cameratest.cpp 内的 image_process_start_idx 和 image_process_end_idx 来设置要调试的图像的索引范围
3. 按 F5 运行调试，可以打断点精细调试每一行代码
4. 经过图像代码处理后的图像保存到 output 文件夹内

将图像代码迁移到车上：
这里的 image.cpp 和 image.h 应该和 "校内赛图像控制代码-new" 内的文件保持完全一致，直接拷贝这两个文件即可。
如果只修改了 Search()，也可以只迁移它
*/

int main()
{
    for (; image_process_start_idx <= image_process_end_idx; ++image_process_start_idx)
    {
        if (!ReadImage())
        {
            continue;
        }

        Search();
        calcTurningCenter();
        Image_DrawOverlay();
        OutputImage();
    }

    return 0;
}