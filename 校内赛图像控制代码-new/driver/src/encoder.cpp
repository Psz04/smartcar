#include "encoder.h"

#include <stdio.h>

#include "board_config.h"
#include "register.h"

/**
 * @brief 初始化编码器。
 * @param encoder 编码器对象。
 * @param pwm_num PWM 通道编号。
 * @param gpio_num 方向 GPIO 编号。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Encoder_Init(Encoder *encoder, int pwm_num, int gpio_num)
{
    if (encoder == NULL) {
        return -1;
    }

    encoder->base_addr = PWM_BASE_ADDR + (unsigned int)pwm_num * PWM_OFFSET;
    encoder->direction_gpio.fd = -1;
    if (Gpio_Init(&encoder->direction_gpio, gpio_num) != 0) {
        return -1;
    }
    if (Gpio_SetDirection(&encoder->direction_gpio, "in") != 0) {
        Gpio_Close(&encoder->direction_gpio);
        return -1;
    }

    encoder->control_buffer = map_register(encoder->base_addr + CONTROL_REG_OFFSET, PAGE_SIZE);
    encoder->low_buffer = map_register(encoder->base_addr + LOW_BUFFER_OFFSET, PAGE_SIZE);
    encoder->full_buffer = map_register(encoder->base_addr + FULL_BUFFER_OFFSET, PAGE_SIZE);
    if (encoder->control_buffer == NULL || encoder->low_buffer == NULL ||
        encoder->full_buffer == NULL) {
        return -1;
    }

    uint32_t control = CNTR_ENABLE_BIT | MEASURE_PULSE_BIT | INT_ENABLE_BIT;
    REG_WRITE(encoder->control_buffer, control);
    return 0;
}

/**
 * @brief 关闭编码器。
 * @param encoder 编码器对象。
 * @retval 无。
 */
void Encoder_Close(Encoder *encoder)
{
    if (encoder == NULL) {
        return;
    }
    Gpio_Close(&encoder->direction_gpio);
}

/**
 * @brief 复位编码器计数器。
 * @param encoder 编码器对象。
 * @retval 无。
 */
void Encoder_Reset(Encoder *encoder)
{
    if (encoder == NULL || encoder->control_buffer == NULL) {
        return;
    }
    uint32_t control = REG_READ(encoder->control_buffer) | COUNTER_RESET_BIT;
    REG_WRITE(encoder->control_buffer, control);
}

/**
 * @brief 读取编码器速度。
 * @param encoder 编码器对象。
 * @retval 编码器速度。
 */
double Encoder_UpdatePulse(Encoder *encoder)
{
    if (encoder == NULL || encoder->full_buffer == NULL) {
        return 0.0;
    }
    uint32_t full = REG_READ(encoder->full_buffer);
    if (full == 0) {
        return 0.0;
    }

    int direction = Gpio_ReadValue(&encoder->direction_gpio) ? 1 : -1;
    return 100000000.0 / (double)full / ENCODER_PPR * (double)direction;
}