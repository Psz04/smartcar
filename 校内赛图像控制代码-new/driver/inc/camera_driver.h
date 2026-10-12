#ifndef DRIVER_CAMERA_DRIVER_H
#define DRIVER_CAMERA_DRIVER_H

#include <opencv2/core.hpp>


int Camera_Init(void);
int Camera_ReadFrame(cv::Mat *frame);
void Camera_Release(void);

#endif /* DRIVER_CAMERA_DRIVER_H */