#ifndef DEBUG_IMAGE_COMPAT_H
#define DEBUG_IMAGE_COMPAT_H

/*
 * 离线调试兼容层。
 *
 * 车端 image.cpp 直接复用了 ZF 库中的 uint8 类型，并且当前版本里
 * variable_init() 的定义与 variableInit() 的调用大小写不一致。
 * 离线调试工程不链接 ZF，因此在这里提供最小兼容定义，保证只需复制
 * 车端的 image.cpp / image.h 过来就能编译。
 *
 * 注意：这里只做兼容，不改变 image.cpp 中的算法逻辑。
 */
#include <cstdint>
#include <iostream>
#include "camera.h"

typedef unsigned char uint8;

#ifndef variable_init
#define variable_init variableInit
#endif

#define DEBUG

#endif /* DEBUG_IMAGE_COMPAT_H */