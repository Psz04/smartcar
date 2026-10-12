#ifndef DRIVER_ENCODER_H
#define DRIVER_ENCODER_H

#include "gpio.h"

/**
 * @brief 编码器对象。
 */
typedef struct {
    unsigned int base_addr;   /**< PWM 寄存器基地址。 */
    Gpio direction_gpio;      /**< 方向 GPIO。 */
    void *low_buffer;         /**< 低电平缓冲寄存器。 */
    void *full_buffer;        /**< 完整周期缓冲寄存器。 */
    void *control_buffer;     /**< 控制寄存器。 */
} Encoder;


int Encoder_Init(Encoder *encoder, int pwm_num, int gpio_num);
void Encoder_Close(Encoder *encoder);
void Encoder_Reset(Encoder *encoder);
double Encoder_UpdatePulse(Encoder *encoder);

#endif /* DRIVER_ENCODER_H */