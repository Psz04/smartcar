#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

/**
 * @brief GPIO 对象。
 */
typedef struct {
    int number;       /**< GPIO 编号。 */
    int fd;           /**< value 文件描述符。 */
    char path[128];   /**< GPIO sysfs 路径。 */
} Gpio;


int Gpio_Init(Gpio *gpio, int number);
void Gpio_Close(Gpio *gpio);
int Gpio_SetDirection(Gpio *gpio, const char *direction);
int Gpio_SetValue(Gpio *gpio, int value);
int Gpio_ReadValue(Gpio *gpio);

#endif /* DRIVER_GPIO_H */