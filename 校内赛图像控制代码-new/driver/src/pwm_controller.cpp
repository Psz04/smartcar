#include "pwm_controller.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

/**
 * @brief 写 sysfs 的辅助函数
 */
static int write_text_file(const char *path, const char *value)
{
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        return -1;
    }
    int ret = fprintf(fp, "%s", value);
    fclose(fp);
    return (ret > 0) ? 0 : -1;
}

/**
 * @brief 初始化 PWM 控制器并导出通道。
 * @param pwm PWM 控制器对象。
 * @param chip pwmchip 编号。
 * @param number pwm 通道编号。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_Init(PwmController *pwm, int chip, int number)
{
    if (pwm == NULL) {
        return -1;
    }

    memset(pwm, 0, sizeof(*pwm));
    pwm->chip = chip;
    pwm->number = number;

    snprintf(pwm->path, sizeof(pwm->path),
             "/sys/class/pwm/pwmchip%d/pwm%d/", chip, number);

    struct stat st;
    if (stat(pwm->path, &st) != 0) {
        char export_path[160];
        char number_text[16];
        snprintf(export_path, sizeof(export_path),
                 "/sys/class/pwm/pwmchip%d/export", chip);
        snprintf(number_text, sizeof(number_text), "%d", number);
        if (write_text_file(export_path, number_text) != 0) {
            return -1;
        }
    }

    return 0;
}

/**
 * @brief 设置 PWM 周期。
 * @param pwm PWM 控制器对象。
 * @param period_ns 周期，单位 ns。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_SetPeriod(PwmController *pwm, unsigned int period_ns)
{
    if (pwm == NULL) {
        return -1;
    }
    char path[192];
    char value[32];
    snprintf(path, sizeof(path), "%speriod", pwm->path);
    snprintf(value, sizeof(value), "%u", period_ns);
    pwm->period = (int)period_ns;
    return write_text_file(path, value);
}

/**
 * @brief 设置 PWM 占空比。
 * @param pwm PWM 控制器对象。
 * @param duty_ns 占空比，单位 ns。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_SetDutyCycle(PwmController *pwm, unsigned int duty_ns)
{
    if (pwm == NULL) {
        return -1;
    }
    char path[192];
    char value[32];
    snprintf(path, sizeof(path), "%sduty_cycle", pwm->path);
    snprintf(value, sizeof(value), "%u", duty_ns);
    pwm->duty_cycle = (int)duty_ns;
    return write_text_file(path, value);
}

/**
 * @brief 设置 PWM 极性。
 * @param pwm PWM 控制器对象。
 * @param polarity 极性字符串，例如 normal 或 inversed。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_SetPolarity(PwmController *pwm, const char *polarity)
{
    if (pwm == NULL || polarity == NULL) {
        return -1;
    }
    char path[192];
    snprintf(path, sizeof(path), "%spolarity", pwm->path);
    return write_text_file(path, polarity);
}

/**
 * @brief 使能 PWM 输出。
 * @param pwm PWM 控制器对象。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_Enable(PwmController *pwm)
{
    if (pwm == NULL) {
        return -1;
    }
    char path[192];
    snprintf(path, sizeof(path), "%senable", pwm->path);
    return write_text_file(path, "1");
}

/**
 * @brief 关闭 PWM 输出。
 * @param pwm PWM 控制器对象。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int PwmController_Disable(PwmController *pwm)
{
    if (pwm == NULL) {
        return -1;
    }
    char path[192];
    snprintf(path, sizeof(path), "%senable", pwm->path);
    return write_text_file(path, "0");
}