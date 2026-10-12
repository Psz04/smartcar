#ifndef DRIVER_BOARD_CONFIG_H
#define DRIVER_BOARD_CONFIG_H

/* camera */
#define CAMERA_DEVICE_NAME              "/dev/video0"
#define CAMERA_FRAME_WIDTH              160
#define CAMERA_FRAME_HEIGHT             120
#define CAMERA_FPS                      60
#define CAMERA_ROI_X                    0
#define CAMERA_ROI_Y                    10
#define CAMERA_ROI_WIDTH                160
#define CAMERA_ROI_HEIGHT               97

#define IMAGE_CENTER_X                 80

/* servo */
#define SERVO_MAX_VALUE                 1220
#define SERVO_MIN_VALUE                 660
#define SERVO_MID_VALUE                 940
#define SERVO_PWM_DEVICE                "/dev/zf_device_pwm_servo"

/* motor */
#define MOTOR_PWM_PERIOD_NS             50000
#define MOTOR_PWM_MID_DUTY              25000
#define MOTOR_MAX_VALUE                 10000
#define MOTOR_MIN_VALUE                 0
#define MOTOR_LEFT_PWM_PIN              2
#define MOTOR_RIGHT_PWM_PIN             1

/* encoder */
#define ENCODER_LEFT_PWM_PIN            0
#define ENCODER_LEFT_DIR_GPIO_PIN       62
#define ENCODER_RIGHT_PWM_PIN           3
#define ENCODER_RIGHT_DIR_GPIO_PIN      61
#define ENCODER_PPR                     100

#define PWM_BASE_ADDR                   0x1611B000u
#define PWM_OFFSET                      0x10u
#define LOW_BUFFER_OFFSET               0x4u
#define FULL_BUFFER_OFFSET              0x8u
#define CONTROL_REG_OFFSET              0xCu
#define CNTR_ENABLE_BIT                 (1u << 0)
#define MEASURE_PULSE_BIT               (1u << 8)
#define INT_ENABLE_BIT                  (1u << 5)
#define COUNTER_RESET_BIT               (1u << 7)

/* keys and switches */
#define SWITCH_1                        "/dev/zf_driver_gpio_switch_1"
#define SWITCH_2                        "/dev/zf_driver_gpio_switch_2"
#define SWITCH_3                        "/dev/zf_driver_gpio_switch_3"
#define SWITCH_4                        "/dev/zf_driver_gpio_switch_4"
#define KEY_BEGIN                       "/dev/zf_driver_gpio_key_begin"
#define KEY_RESET                       "/dev/zf_driver_gpio_key_reset"

/* battery voltage ADC */
#define ADC_REG_PATH                    "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
#define ADC_SCALE_PATH                  "/sys/bus/iio/devices/iio:device0/in_voltage_scale"
#define BATTERY_VOLTAGE_DIVIDER_RATIO   11.2

#define UPPER_COMPUTER_IP               "192.168.3.9"   // 上位机电脑 IP,连接 USTBSMARTCAR 后在命令提示符里输入 ipconfig 查看无线局域网适配器 WLAN 的 IPv4 地址
#define UPPER_COMPUTER_PORT             3000
#define CONTROL_PERIOD_MS               5

#endif /* DRIVER_BOARD_CONFIG_H */
