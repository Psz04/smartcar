#include "gpio.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

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
 * @brief 初始化并导出 GPIO。
 * @param gpio GPIO 对象。
 * @param number GPIO 编号。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Gpio_Init(Gpio *gpio, int number)
{
    if (gpio == NULL) {
        return -1;
    }

    memset(gpio, 0, sizeof(*gpio));
    gpio->number = number;
    gpio->fd = -1;
    snprintf(gpio->path, sizeof(gpio->path),
             "/sys/class/gpio/gpio%d", number);

    struct stat st;
    if (stat(gpio->path, &st) != 0) {
        char value[16];
        snprintf(value, sizeof(value), "%d", number);
        if (write_text_file("/sys/class/gpio/export", value) != 0) {
            return -1;
        }
    }

    char value_path[160];
    snprintf(value_path, sizeof(value_path), "%s/value", gpio->path);
    gpio->fd = open(value_path, O_RDWR);
    if (gpio->fd < 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief 关闭 GPIO 文件描述符。
 * @param gpio GPIO 对象。
 * @retval 无。
 */
void Gpio_Close(Gpio *gpio)
{
    if (gpio == NULL) {
        return;
    }
    if (gpio->fd >= 0) {
        close(gpio->fd);
        gpio->fd = -1;
    }
}

/**
 * @brief 设置 GPIO 方向。
 * @param gpio GPIO 对象。
 * @param direction 方向字符串，in 或 out。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Gpio_SetDirection(Gpio *gpio, const char *direction)
{
    if (gpio == NULL || direction == NULL) {
        return -1;
    }
    char path[160];
    snprintf(path, sizeof(path), "%s/direction", gpio->path);
    return write_text_file(path, direction);
}

/**
 * @brief 设置 GPIO 输出电平。
 * @param gpio GPIO 对象。
 * @param value 0 表示低电平，非 0 表示高电平。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Gpio_SetValue(Gpio *gpio, int value)
{
    if (gpio == NULL || gpio->fd < 0) {
        return -1;
    }
    const char *text = (value != 0) ? "1" : "0";
    lseek(gpio->fd, 0, SEEK_SET);
    int ret = (int)write(gpio->fd, text, 1);
    return (ret == 1) ? 0 : -1;
}

/**
 * @brief 读取 GPIO 输入电平。
 * @param gpio GPIO 对象。
 * @retval 0 低电平。
 * @retval 1 高电平。
 */
int Gpio_ReadValue(Gpio *gpio)
{
    if (gpio == NULL || gpio->fd < 0) {
        return 0;
    }
    char text = 0;
    lseek(gpio->fd, 0, SEEK_SET);
    if (read(gpio->fd, &text, 1) != 1) {
        return 0;
    }
    return (text == '1') ? 1 : 0;
}