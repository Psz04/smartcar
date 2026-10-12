#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "protocol.h"

/**
 * @brief PID 控制类型。
 */
typedef enum {
    PID_POSITION = 0,     // 位置式 PID
    PID_INCREMENTAL = 1   // 增量式 PID
} PidMode;

/**
 * @brief PID 控制器结构体。
 */
typedef struct {
    PidMode mode;        // PID 类型
    float kp;            // 比例系数
    float ki;            // 积分系数
    float kd;            // 微分系数
    float dt;            // 控制周期，单位秒
    float target;        // 目标值
    float actual;        // 反馈值
    float error;         // 本次误差
    float last_error;    // 上一次误差
    float prev_error;    // 上上次误差
    float integral;      // 位置式 PID 积分项
    float output;        // PID 输出
    float output_limit;  // 输出限幅，0 表示不限幅
    float integral_limit;// 积分限幅，0 表示不限幅
} PidController;


void Pid_Init(PidController *pid, PidMode mode);
void Pid_Reset(PidController *pid);
float Pid_CalcPosition(PidController *pid, float target, float actual);
float Pid_CalcIncremental(PidController *pid, float target, float actual);
void Control_Init(void);
void Control_Update5ms(void);
void Control_SetRunState(int running);
int Control_IsRunning(void);
void Control_SetTurningCenter(int center);
int Control_GetTurningCenter(void);
void Control_SetMotorBasePwm(int pwm);
void Control_SetMotorBaseSpeed(int speed);
void Control_SetMotorDiff(int diff_k);
void Control_SetServoPid(float kp, float ki, float kd);
void Control_SetMotorPid(float kp, float ki, float kd);
void Control_ServoByTurningCenter(int center);
void Control_MotorByPwm(int base_pwm, float servo_output);
void Control_MotorBySpeed(int target_speed, float servo_output);
int Control_HandleCommand(const ProtocolCommand *command);

#endif /* APP_CONTROL_H */