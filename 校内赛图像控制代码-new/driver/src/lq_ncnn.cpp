/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-03-03
@修改内容：整理代码风格，与其他驱动库保持一致
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "lq_ncnn.h"

#include <cstdio>
#include <cfloat>
#include <cmath>

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LQ_NCNN::LQ_NCNN()
 * @功能说明：NCNN类的构造函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：LQ_NCNN ncnn;
 * @备注说明：初始化默认参数
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
// LQ_NCNN::LQ_NCNN()
//     : m_initialized(false)
//     , m_input_width(64)
//     , m_input_height(64)
//     , m_input_name("in0")
//     , m_output_name("out0")
// {
//     // 默认使用ImageNet标准归一化
//     m_mean_vals[0] = 123.675f;
//     m_mean_vals[1] = 116.28f;
//     m_mean_vals[2] = 103.53f;
//     m_norm_vals[0] = 0.01712475f;
//     m_norm_vals[1] = 0.017507f;
//     m_norm_vals[2] = 0.01742919f;
// }
LQ_NCNN::LQ_NCNN()
    : m_initialized(false)
    , m_input_width(64)
    , m_input_height(64)
    , m_input_name("in0")
    , m_output_name("out0")
{
    // ImageNet标准归一化 (mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
    m_mean_vals[0] = 123.675f;
    m_mean_vals[1] = 116.28f;
    m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 0.017125f;
    m_norm_vals[1] = 0.017507f;
    m_norm_vals[2] = 0.017429f;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：LQ_NCNN::~LQ_NCNN()
 * @功能说明：NCNN类的析构函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：创建的对象生命周期结束后系统自动调用
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
LQ_NCNN::~LQ_NCNN()
{
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：bool LQ_NCNN::LoadModel(const char* param_path, const char* bin_path)
 * @功能说明：加载NCNN模型
 * @参数说明：param_path : 模型参数文件路径(.param)
 * @参数说明：bin_path   : 模型权重文件路径(.bin)
 * @函数返回：true:成功 false:失败
 * @调用方法：ncnn.LoadModel("model.param", "model.bin");
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
bool LQ_NCNN::LoadModel(const char* param_path, const char* bin_path)
{
    if (!param_path || !bin_path) {
        printf("NCNN: 模型路径为空\n");
        return false;
    }
    m_param_path = param_path;
    m_bin_path = bin_path;
    return Init();
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetModelPath(const char* param_path, const char* bin_path)
 * @功能说明：设置模型文件路径（不立即加载）
 * @参数说明：param_path : 模型参数文件路径(.param)
 * @参数说明：bin_path   : 模型权重文件路径(.bin)
 * @函数返回：无
 * @调用方法：ncnn.SetModelPath("model.param", "model.bin");
 * @备注说明：需在Init()之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetModelPath(const char* param_path, const char* bin_path)
{
    if (param_path) m_param_path = param_path;
    if (bin_path) m_bin_path = bin_path;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：bool LQ_NCNN::Init()
 * @功能说明：初始化NCNN模型
 * @参数说明：无
 * @函数返回：true:成功 false:失败
 * @调用方法：内部调用，由LoadModel触发
 * @备注说明：加载模型文件和参数
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
bool LQ_NCNN::Init()
{
    m_net.clear();
    m_initialized = false;

    m_net.opt.use_vulkan_compute = false;
    m_net.opt.num_threads = 1;

    if (m_param_path.empty()) {
        printf("NCNN: 参数文件路径未设置\n");
        return false;
    }
    printf("NCNN: 开始加载参数 %s\n", m_param_path.c_str());
    fflush(stdout);
    if (m_net.load_param(m_param_path.c_str()) != 0) {
        printf("NCNN: 加载参数文件失败: %s\n", m_param_path.c_str());
        return false;
    }
    printf("NCNN: 参数加载成功\n");
    fflush(stdout);

    if (m_bin_path.empty()) {
        printf("NCNN: 模型文件路径未设置\n");
        return false;
    }
    printf("NCNN: 开始加载模型 %s\n", m_bin_path.c_str());
    fflush(stdout);
    if (m_net.load_model(m_bin_path.c_str()) != 0) {
        printf("NCNN: 加载模型文件失败: %s\n", m_bin_path.c_str());
        return false;
    }
    printf("NCNN: 模型加载成功\n");
    fflush(stdout);

    m_initialized = true;
    printf("NCNN: 模型初始化成功 (%s)\n", m_param_path.c_str());
    int layer_count = (int)m_net.layers().size();
    int blob_count = (int)m_net.blobs().size();
    printf("NCNN: ====== 模型信息 ======\n");
    printf("NCNN: 参数文件: %s\n", m_param_path.c_str());
    printf("NCNN: 权重文件: %s\n", m_bin_path.c_str());
    printf("NCNN: 模型层数: %d\n", layer_count);
    printf("NCNN: Blob数量: %d\n", blob_count);
    printf("NCNN: 输入尺寸: %dx%d\n", m_input_width, m_input_height);
    printf("NCNN: 输入名称: %s  输出名称: %s\n", m_input_name.c_str(), m_output_name.c_str());
    printf("NCNN: ========================\n");
    fflush(stdout);
    return true;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetInputSize(int width, int height)
 * @功能说明：设置输入尺寸
 * @参数说明：width  : 输入宽度
 * @参数说明：height : 输入高度
 * @函数返回：无
 * @调用方法：ncnn.SetInputSize(224, 224);
 * @备注说明：应在LoadModel之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetInputSize(int width, int height)
{
    m_input_width = width;
    m_input_height = height;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetLabels(const std::vector<std::string>& labels)
 * @功能说明：设置类别标签
 * @参数说明：labels : 类别名称列表
 * @函数返回：无
 * @调用方法：ncnn.SetLabels({"cat", "dog", "bird"});
 * @备注说明：应在LoadModel之前调用
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetLabels(const std::vector<std::string>& labels)
{
    m_labels = labels;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
 * @功能说明：设置归一化参数
 * @参数说明：mean_vals : 均值（BGR三通道）
 * @参数说明：norm_vals : 缩放因子（1/标准差）
 * @函数返回：无
 * @调用方法：ncnn.SetNormalize(mean, norm);
 * @备注说明：默认为 [0,1] 归一化，ImageNet预训练模型需设置为ImageNet均值
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
{
    m_mean_vals[0] = mean_vals[0];
    m_mean_vals[1] = mean_vals[1];
    m_mean_vals[2] = mean_vals[2];
    m_norm_vals[0] = norm_vals[0];
    m_norm_vals[1] = norm_vals[1];
    m_norm_vals[2] = norm_vals[2];
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：std::string LQ_NCNN::Infer(const cv::Mat& bgr_image)
 * @功能说明：推理函数
 * @参数说明：bgr_image : 输入BGR图像
 * @函数返回：推理结果（类别名称或编号），失败返回空字符串
 * @调用方法：std::string result = ncnn.Infer(image);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
std::string LQ_NCNN::Infer(const cv::Mat& bgr_image)
{
    if (!m_initialized) {
        printf("NCNN: 模型未初始化\n");
        return "";
    }
    if (bgr_image.empty()) {
        printf("NCNN: 输入图像为空\n");
        return "";
    }

    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(m_input_width, m_input_height), 0, 0, cv::INTER_LANCZOS4);
    m_input = ncnn::Mat::from_pixels(
        resized.data, ncnn::Mat::PIXEL_BGR2RGB,
        m_input_width, m_input_height);

    m_input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    ncnn::Extractor ex = m_net.create_extractor();
    ex.input(m_input_name.c_str(), m_input);

    if (ex.extract(m_output_name.c_str(), m_output) != 0) {
        printf("NCNN: 推理失败\n");
        return "";
    }

    int class_id = Argmax(m_output);
    if (class_id < 0) {
        printf("NCNN: 获取类别失败\n");
        return "";
    }

    if (class_id < static_cast<int>(m_labels.size())) {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：std::string LQ_NCNN::Infer(const cv::Mat& bgr_image, float& confidence)
 * @功能说明：推理函数（带置信度输出）
 * @参数说明：bgr_image   : 输入BGR图像
 * @参数说明：confidence  : 输出置信度（softmax概率）
 * @函数返回：推理结果（类别名称或编号），失败返回空字符串
 * @调用方法：float conf; std::string result = ncnn.Infer(image, conf);
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
std::string LQ_NCNN::Infer(const cv::Mat& bgr_image, float& confidence)
{
    confidence = 0.0f;
    if (!m_initialized) {
        printf("NCNN: 模型未初始化\n");
        return "";
    }
    if (bgr_image.empty()) {
        printf("NCNN: 输入图像为空\n");
        return "";
    }

    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(m_input_width, m_input_height), 0, 0, cv::INTER_LANCZOS4);
    m_input = ncnn::Mat::from_pixels(
        resized.data, ncnn::Mat::PIXEL_BGR2RGB,
        m_input_width, m_input_height);

    m_input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    ncnn::Extractor ex = m_net.create_extractor();
    ex.input(m_input_name.c_str(), m_input);

    if (ex.extract(m_output_name.c_str(), m_output) != 0) {
        printf("NCNN: 推理失败\n");
        return "";
    }

    int class_id = Argmax(m_output);
    if (class_id < 0) {
        printf("NCNN: 获取类别失败\n");
        return "";
    }

    confidence = SoftmaxConfidence(m_output, class_id);

    if (class_id < static_cast<int>(m_labels.size())) {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：float LQ_NCNN::SoftmaxConfidence(const ncnn::Mat& logits, int index)
 * @功能说明：计算指定类别的softmax置信度
 * @参数说明：logits : 模型原始输出
 * @参数说明：index  : 目标类别索引
 * @函数返回：置信度（0~1）
 * @调用方法：内部调用
 * @备注说明：softmax = exp(x[i]) / sum(exp(x[j]))
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float LQ_NCNN::SoftmaxConfidence(const ncnn::Mat& logits, int index)
{
    const float* data = (const float*)logits.data;
    int total = logits.w * logits.h * logits.c;
    if (total <= 0 || index < 0 || index >= total) {
        return 0.0f;
    }

    float max_val = -FLT_MAX;
    for (int i = 0; i < total; ++i) {
        if (data[i] > max_val) max_val = data[i];
    }

    float sum = 0.0f;
    for (int i = 0; i < total; ++i) {
        sum += expf(data[i] - max_val);
    }

    return expf(data[index] - max_val) / sum;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：int LQ_NCNN::Argmax(const ncnn::Mat& logits)
 * @功能说明：获取最大概率类别索引
 * @参数说明：logits : 模型输出
 * @函数返回：类别索引，失败返回-1
 * @调用方法：内部调用
 * @备注说明：遍历所有维度 w*h*c
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_NCNN::Argmax(const ncnn::Mat& logits)
{
    const float* data = (const float*)logits.data;
    int total = logits.w * logits.h * logits.c;
    if (total <= 0) {
        return -1;
    }

    int best_index = 0;
    float best_value = -FLT_MAX;

    for (int i = 0; i < total; ++i) {
        if (data[i] > best_value) {
            best_value = data[i];
            best_index = i;
        }
    }
    return best_index;
}
