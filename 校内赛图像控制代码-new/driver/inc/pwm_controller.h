#ifndef DRIVER_PWM_CONTROLLER_H
#define DRIVER_PWM_CONTROLLER_H

#include <stddef.h>

/**
 * @brief PWM 控制器对象。
 */
typedef struct {
    char path[160];   /**< PWM sysfs 路径。 */
    int chip;         /**< pwmchip 编号。 */
    int number;       /**< pwm 通道编号。 */
    int period;       /**< 当前周期，单位 ns。 */
    int duty_cycle;   /**< 当前占空比，单位 ns。 */
} PwmController;


int PwmController_Init(PwmController *pwm, int chip, int number);
int PwmController_SetPeriod(PwmController *pwm, unsigned int period_ns);
int PwmController_SetDutyCycle(PwmController *pwm, unsigned int duty_ns);
int PwmController_SetPolarity(PwmController *pwm, const char *polarity);
int PwmController_Enable(PwmController *pwm);
int PwmController_Disable(PwmController *pwm);

#endif /* DRIVER_PWM_CONTROLLER_H */