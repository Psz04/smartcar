#ifndef DRIVER_PROTOCOL_H
#define DRIVER_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_MAX_TYPE_LEN       32  // 指令类型字符串最大长度
#define PROTOCOL_MAX_PARAM_COUNT    8   // 指令参数最大数量
#define PROTOCOL_MAX_PARAM_LEN      32  // 指令参数最大长度

/**
 * @brief 解析后的上位机指令。
 */
typedef struct {
    char type[PROTOCOL_MAX_TYPE_LEN];   /**< 指令类型，例如 START、SET_FPS、SERVO_PID。 */
    int param_count;                    /**< 参数个数。 */
    char params[PROTOCOL_MAX_PARAM_COUNT][PROTOCOL_MAX_PARAM_LEN]; /**< 参数字符串。 */
} ProtocolCommand;


int Protocol_ParseCommand(const char *text, ProtocolCommand *command);
int Protocol_BuildImageMeta(int turning_center, int fps, char *out, size_t out_size);
int Protocol_BuildImageFrame(const uint8_t *jpeg, size_t jpeg_len, const char *meta,
                             uint8_t *out, size_t out_size);

#endif /* DRIVER_PROTOCOL_H */