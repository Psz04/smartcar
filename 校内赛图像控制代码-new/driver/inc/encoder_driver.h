#ifndef DRIVER_ENCODER_DRIVER_H
#define DRIVER_ENCODER_DRIVER_H


int Encoder_DriverInit(void);
void Encoder_Update(void);
int Encoder_GetLeftSpeed(void);
int Encoder_GetRightSpeed(void);

#endif /* DRIVER_ENCODER_DRIVER_H */