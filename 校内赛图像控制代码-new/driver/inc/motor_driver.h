#ifndef DRIVER_MOTOR_DRIVER_H
#define DRIVER_MOTOR_DRIVER_H


int Motor_Init(void);
void Motor_SetPwm(int left_pwm, int right_pwm);
void Motor_Stop(void);

#endif /* DRIVER_MOTOR_DRIVER_H */