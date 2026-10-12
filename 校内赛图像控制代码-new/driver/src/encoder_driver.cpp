#include "encoder_driver.h"

#include "board_config.h"
#include "encoder.h"

static Encoder g_encoder_left;
static Encoder g_encoder_right;
static int g_left_speed = 0;
static int g_right_speed = 0;

/**
 * @brief 初始化左右编码器。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Encoder_DriverInit(void)
{
    if (Encoder_Init(&g_encoder_left, ENCODER_LEFT_PWM_PIN,
                     ENCODER_LEFT_DIR_GPIO_PIN) != 0) {
        return -1;
    }
    if (Encoder_Init(&g_encoder_right, ENCODER_RIGHT_PWM_PIN,
                     ENCODER_RIGHT_DIR_GPIO_PIN) != 0) {
        return -1;
    }
    g_left_speed = 0;
    g_right_speed = 0;
    return 0;
}

/**
 * @brief 更新左右编码器速度。
 * @retval 无。
 */
void Encoder_Update(void)
{
    // 如果编码器方向反了，可以改这里的符号
    g_left_speed = (int)(Encoder_UpdatePulse(&g_encoder_left) * -2.0);
    g_right_speed = (int)(Encoder_UpdatePulse(&g_encoder_right) * 2.0);
}

/**
 * @brief 获取左编码器速度。
 * @retval 左编码器速度。
 */
int Encoder_GetLeftSpeed(void)
{
    return g_left_speed;
}

/**
 * @brief 获取右编码器速度。
 * @retval 右编码器速度。
 */
int Encoder_GetRightSpeed(void)
{
    return g_right_speed;
}