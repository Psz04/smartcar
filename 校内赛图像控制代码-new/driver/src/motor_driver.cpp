#include "motor_driver.h"

#include "board_config.h"
#include "pwm_controller.h"

static PwmController g_motor_left;
static PwmController g_motor_right;

static int motor_channel_init(PwmController *pwm, int pin)
{
    if (PwmController_Init(pwm, pin, 0) != 0) {
        return -1;
    }
    if (PwmController_SetPeriod(pwm, MOTOR_PWM_PERIOD_NS) != 0) {
        return -1;
    }
    if (PwmController_SetDutyCycle(pwm, MOTOR_PWM_MID_DUTY) != 0) {
        return -1;
    }
    if (PwmController_SetPolarity(pwm, "inversed") != 0) {
        return -1;
    }
    if (PwmController_Enable(pwm) != 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief 初始化左右电机 PWM。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Motor_Init(void)
{
    if (motor_channel_init(&g_motor_left, MOTOR_LEFT_PWM_PIN) != 0) {
        return -1;
    }
    if (motor_channel_init(&g_motor_right, MOTOR_RIGHT_PWM_PIN) != 0) {
        return -1;
    }
    Motor_Stop();
    return 0;
}

/**
 * @brief PWM 值映射，-10000 ~ 10000 -> 0 ~ 50000
 */
static unsigned int motor_pwm_to_duty(int pwm)
{
    long duty = (long)pwm * 5 / 2 + MOTOR_PWM_MID_DUTY;
    if (duty < 0) {
        duty = 0;
    }
    if (duty > MOTOR_PWM_PERIOD_NS) {
        duty = MOTOR_PWM_PERIOD_NS;
    }
    return (unsigned int)duty;
}

/**
 * @brief 设置左右电机 PWM。
 * @param left_pwm 左电机 PWM，范围 -10000~10000。
 * @param right_pwm 右电机 PWM，范围 -10000~10000。
 * @retval 无。
 */
void Motor_SetPwm(int left_pwm, int right_pwm)
{
    PwmController_SetDutyCycle(&g_motor_left, motor_pwm_to_duty(left_pwm));
    PwmController_SetDutyCycle(&g_motor_right, motor_pwm_to_duty(right_pwm));
}

/**
 * 
 * @brief 停止左右电机。
 * @retval 无。
 */
void Motor_Stop(void)
{
    Motor_SetPwm(0, 0);
}