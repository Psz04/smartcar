#include "protocol.h"

#include <stdio.h>
#include <string.h>

/**
 * @brief 消除字符串两端的多余空字符
 */
static void trim_text(char *text)
{
    if (text == NULL) {
        return;
    }

    char *start = text;
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
        ++start;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }

    size_t len = strlen(text);
    while (len > 0) {
        char ch = text[len - 1];
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            break;
        }
        text[len - 1] = '\0';
        --len;
    }
}

/**
 * @brief 解析 TYPE, 参数1, 参数2, ... 格式的文本指令。
 * @param text 输入文本。
 * @param command 输出解析结果。
 * @retval 0 成功。
 * @retval -1 失败。
 */
int Protocol_ParseCommand(const char *text, ProtocolCommand *command)
{
    if (text == NULL || command == NULL) {
        return -1;
    }

    memset(command, 0, sizeof(*command));

    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s", text);
    trim_text(buffer);
    if (buffer[0] == '\0') {
        return -1;
    }

    char *token = strtok(buffer, ",");
    if (token == NULL) {
        return -1;
    }
    trim_text(token);
    snprintf(command->type, sizeof(command->type), "%s", token);

    while ((token = strtok(NULL, ",")) != NULL) {
        trim_text(token);
        if (command->param_count >= PROTOCOL_MAX_PARAM_COUNT) {
            break;
        }
        snprintf(command->params[command->param_count],
                 sizeof(command->params[command->param_count]), "%s", token);
        command->param_count++;
    }

    return 0;
}

/**
 * @brief 构造图像帧的 meta 文本。
 * @param turning_center 转向中心。
 * @param fps 图传帧率。
 * @param out 输出缓冲区。
 * @param out_size 输出缓冲区大小。
 * @retval >=0 写入的字符数。
 * @retval -1 失败。
 */
int Protocol_BuildImageMeta(int turning_center, int fps, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return -1;
    }
    int written = snprintf(out, out_size, "type=IMAGE;mid=%d;fps=%d",
                           turning_center, fps);
    if (written < 0 || (size_t)written >= out_size) {
        return -1;
    }
    return written;
}

/**
 * @brief 构造车端图像二进制帧。
 * @param jpeg JPEG 数据。
 * @param jpeg_len JPEG 数据长度。
 * @param meta meta 文本。
 * @param out 输出缓冲区。
 * @param out_size 输出缓冲区大小。
 * @retval >=0 写入的字节数。
 * @retval -1 失败。
 */
int Protocol_BuildImageFrame(const uint8_t *jpeg, size_t jpeg_len, const char *meta, uint8_t *out, size_t out_size)
{
    if (jpeg == NULL || meta == NULL || out == NULL) {
        return -1;
    }

    size_t meta_len = strlen(meta);
    size_t total = 1 + 4 + meta_len + jpeg_len;
    if (total > out_size) {
        return -1;
    }

    size_t index = 0;
    out[index++] = 0x01; /* TYPE = IMAGE */

    uint32_t len = (uint32_t)meta_len;
    out[index++] = (uint8_t)((len >> 24) & 0xFF);
    out[index++] = (uint8_t)((len >> 16) & 0xFF);
    out[index++] = (uint8_t)((len >> 8) & 0xFF);
    out[index++] = (uint8_t)(len & 0xFF);

    memcpy(out + index, meta, meta_len);
    index += meta_len;
    memcpy(out + index, jpeg, jpeg_len);
    index += jpeg_len;

    return (int)index;
}