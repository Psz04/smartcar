#include "servo_driver.h"

#include "board_config.h"
#include "zf_driver_pwm.h"

/**
 * @brief 初始化舵机。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Servo_Init(void)
{
    Servo_SetPulse(SERVO_MID_VALUE);
    return 0;
}

/**
 * @brief 设置舵机脉宽。
 * @param pulse 舵机脉宽值，范围由板级配置决定。
 * @retval 无。
 */
void Servo_SetPulse(unsigned int pulse)
{
    if (pulse > SERVO_MAX_VALUE) {
        pulse = SERVO_MAX_VALUE;
    }
    if (pulse < SERVO_MIN_VALUE) {
        pulse = SERVO_MIN_VALUE;
    }
    pwm_set_duty(SERVO_PWM_DEVICE, (uint16)pulse);
}