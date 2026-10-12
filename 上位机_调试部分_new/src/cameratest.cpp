#include "camera.h"
#include "image.h"

#include <cerrno>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

// 输入/输出目录
#ifndef INPUT_DIR
#define INPUT_DIR "/home/smartcar/input"
#endif

#ifndef OUTPUT_DIR
#define OUTPUT_DIR "/home/smartcar/output"
#endif

namespace
{
const std::string kInputDir = INPUT_DIR;
const std::string kOutputDir = OUTPUT_DIR;

std::string MakeImagePath(const std::string& dir, int index)
{
    return dir + "/" + std::to_string(index) + ".jpg";
}

bool EnsureDirectory(const std::string& path)
{
    if (path.empty())
    {
        return false;
    }

#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}
} // namespace

cv::Mat grayFrame;
cv::Mat binaryFrame;

// 修改这里设置要调试的帧索引范围，包含首尾。
int image_process_start_idx = 0;
int image_process_end_idx = 1007;

bool ReadImage()
{
    const std::string inputPath = MakeImagePath(kInputDir, image_process_start_idx);
    cv::Mat src = cv::imread(inputPath, cv::IMREAD_GRAYSCALE);

    if (src.empty())
    {
        std::cerr << "[ReadImage] 无法读取图像: " << inputPath << std::endl;
        grayFrame.release();
        binaryFrame.release();
        return false;
    }

    // 统一到图像数组尺寸，保证后续坐标与 g_VideoImageData 一致。
    if (src.cols != MAX_VIDEO_POINT || src.rows != MAX_VIDEO_LINE)
    {
        const int interpolation = (src.cols > MAX_VIDEO_POINT || src.rows > MAX_VIDEO_LINE)
                                      ? cv::INTER_AREA
                                      : cv::INTER_LINEAR;
        cv::resize(src, grayFrame, cv::Size(MAX_VIDEO_POINT, MAX_VIDEO_LINE), 0, 0, interpolation);
    }
    else
    {
        grayFrame = src;
    }

    // 生成 Otsu 二值图和阈值，供 Search() 使用。
    binary_threshold = (uint8_t)cv::threshold(grayFrame, binaryFrame, 0, 255,
                                              cv::THRESH_BINARY | cv::THRESH_OTSU);

    // g_VideoImageData 的行号从图像底部向上增大，因此这里做一次翻转拷贝。
    for (int row = 0; row < MAX_VIDEO_LINE; ++row)
    {
        const unsigned char* srcRow = grayFrame.ptr<unsigned char>(MAX_VIDEO_LINE - 1 - row);
        uint8_t* dstRow = g_VideoImageData[row];

        for (int col = 0; col < MAX_VIDEO_POINT; ++col)
        {
            dstRow[col] = srcRow[col];
        }
    }

    std::cout << "[ReadImage] " << inputPath << std::endl;
    return true;
}
void OutputImage()
{
    if (grayFrame.empty())
    {
        std::cerr << "[OutputImage] 没有可保存的图像" << std::endl;
        return;
    }

    if (!EnsureDirectory(kOutputDir))
    {
        std::cerr << "[OutputImage] 无法创建输出目录: " << kOutputDir << std::endl;
        return;
    }

    const cv::Mat *overlay = Image_GetOverlayFrame();
    const cv::Mat& outputImage =
        (overlay != NULL && !overlay->empty()) ? *overlay : grayFrame;
    const std::string outputPath = MakeImagePath(kOutputDir, image_process_start_idx);

    try
    {
        if (cv::imwrite(outputPath, outputImage))
        {
            std::cout << "[OutputImage] " << outputPath << " 保存成功" << std::endl;
        }
        else
        {
            std::cerr << "[OutputImage] " << outputPath << " 保存失败" << std::endl;
        }
    }
    catch (const cv::Exception& error)
    {
        std::cerr << "[OutputImage] OpenCV 异常: " << error.what() << std::endl;
    }
}
