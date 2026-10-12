#ifndef __LQ_NCNN_H__
#define __LQ_NCNN_H__

#include "net.h"
#include "mat.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class LQ_NCNN
{
public:
    LQ_NCNN();
    ~LQ_NCNN();

    bool LoadModel(const char* param_path, const char* bin_path);

    void SetModelPath(const char* param_path, const char* bin_path);
    void SetInputSize(int width, int height);
    void SetLabels(const std::vector<std::string>& labels);
    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);

    bool Init();

    std::string Infer(const cv::Mat& bgr_image);
    std::string Infer(const cv::Mat& bgr_image, float& confidence);

    bool IsInitialized() const { return m_initialized; }

private:
    int Argmax(const ncnn::Mat& logits);
    float SoftmaxConfidence(const ncnn::Mat& logits, int index);

    ncnn::Net m_net;
    ncnn::Mat m_input;
    ncnn::Mat m_output;

    bool m_initialized;
    int m_input_width;
    int m_input_height;

    std::string m_param_path;
    std::string m_bin_path;
    std::string m_input_name;
    std::string m_output_name;

    std::vector<std::string> m_labels;

    float m_mean_vals[3];
    float m_norm_vals[3];
};

#endif
