#include "control.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "board_config.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "zf_driver_gpio.h"

#define MOTOR_CLOSED_LOOP false  // 电机闭环控制标志，如果编码器可用，则设为 true，否则为 false

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* ---------------- 发车按键（编程题 1） ---------------- */
#define KEY_PRESSED_LEVEL       0       // 考核说明：按下时 gpio_get_level(KEY_BEGIN) 为 0，松开为 1
#define KEY_START_HOLD_MS       2000    // 需要持续按下的时间，单位毫秒
#define KEY_START_HOLD_TICKS    (KEY_START_HOLD_MS / CONTROL_PERIOD_MS) // 换算成 5ms 中断次数：400 次

/* ---------------- 电机差速（编程题 3） ---------------- */
#define MOTOR_DIFF_MAX          50      // 上位机设定的差速因子限幅，防止调飞

static PidController g_servo_pid; // 舵机 PID 
static volatile int g_running = 0; // 运行标志
static volatile int g_turning_center = IMAGE_CENTER_X; // 转向中心
static int g_key_hold_ticks = 0; // 发车按键已持续按下的中断次数
#if MOTOR_CLOSED_LOOP
static PidController g_motorL_pid; // 左电机 PID
static PidController g_motorR_pid; // 右电机 PID
static volatile int g_motor_base_speed = 100; // 电机基础目标速度
#else
static volatile int g_motor_base_pwm = 2000;   // 开环控制 PWM 基础值
#endif
static int g_motor_diff_k = 3; // 电机差速因子


/**
 * @brief 浮点数限幅函数
 */
static float clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

/**
 * @brief 发车按键检测（编程题 1）。
 * @note  在 5ms 周期控制中断中调用，用计数代替延时：
 *        按键按下时每次中断计 1 个 tick，累计到 KEY_START_HOLD_TICKS（400 次 = 2s）
 *        即置运行标志发车；中途松开则计数清零、重新计时。
 *        全程不使用 delay / sleep，既不阻塞控制中断，也不改变上位机 START 指令的
 *        原有发车路径（START 指令仍然直接调用 Control_SetRunState(1)）。
 */
static void Control_CheckStartKey(void)
{
    if(gpio_get_level(KEY_BEGIN) == KEY_PRESSED_LEVEL) // 按键按下
    {
        if(g_key_hold_ticks < KEY_START_HOLD_TICKS)
        {
            g_key_hold_ticks++;
        }
        if(g_key_hold_ticks >= KEY_START_HOLD_TICKS) // 已持续按下 2s，发车
        {
            Control_SetRunState(1);
        }
    }
    else // 松开，重新计时
    {
        g_key_hold_ticks = 0;
    }
}

/**
 * @brief 初始化 PID 控制器。
 * @param pid PID 控制器对象。
 * @param mode PID 类型。
 */
void Pid_Init(PidController *pid, PidMode mode)
{
    if(pid == NULL)
    {
        return;
    }
    memset(pid, 0, sizeof(*pid));
    pid->mode = mode;
    pid->dt = 0.005f;
    pid->output_limit = 0.0f;   // 默认不做输出限幅
    pid->integral_limit = 0.0f; // 默认不做积分限幅
}

/**
 * @brief 复位 PID 运行状态。
 * @param pid PID 控制器对象。
 */
void Pid_Reset(PidController *pid)
{
    if(pid == NULL)
    {
        return;
    }
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 计算位置式 PID。
 * @param pid PID 控制器对象。
 * @param target 目标值。
 * @param actual 反馈值。
 * @retval PID 输出值。
 */
float Pid_CalcPosition(PidController *pid, float target, float actual)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    float dt = (pid->dt > 0.000001f) ? pid->dt : 0.005f;
    // 赋值
    pid->target = target;
    pid->actual = actual;
    pid->error = target - actual;
    // 积分项计算
    pid->integral += pid->error * dt;
    // 积分限幅
    if(pid->integral_limit > 0.0f)
    {
        pid->integral = clamp_float(pid->integral, -pid->integral_limit, pid->integral_limit);
    }
    // 微分项计算
    float derivative = (pid->error - pid->last_error) / dt;
    // 相加得输出
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;
    // 输出限幅
    if(pid->output_limit > 0.0f)
    {
        pid->output = clamp_float(pid->output, -pid->output_limit, pid->output_limit);
    }
    // 这次误差 -> 上次误差
    pid->last_error = pid->error;
    return pid->output;
}

/**
 * @brief 计算增量式 PID。
 * @param pid PID 控制器对象。
 * @param target 目标值。
 * @param actual 反馈值。
 * @retval PID 累加输出值。
 */
float Pid_CalcIncremental(PidController *pid, float target, float actual)
{
    if(pid == NULL)
    {
        return 0.0f;
    }

    float dt = (pid->dt > 0.000001f) ? pid->dt : 0.005f;

    /* 取三次误差：e0 = e[k-2]，e1 = e[k-1]，e2 = e[k]（本次） */
    const float e0 = pid->prev_error; // 上上次误差
    const float e1 = pid->last_error; // 上次误差
    const float e2 = target - actual; // 本次误差

    // 增量式 PID：Δu = Kp*(e2-e1) + Ki*e2*dt + Kd*(e2-2*e1+e0)/dt
    float delta = pid->kp * (e2 - e1)
                + pid->ki * e2 * dt
                + pid->kd * (e2 - 2.0f * e1 + e0) / dt;

    /* 积分限幅（对应问答题 1）：
       增量式 PID 没有显式的积分累加器，积分作用隐含在"输出的累加"里，
       因此限幅分两级做 ——
         1) 限"单次增量"：用 integral_limit 限住 Δu，防止一次突变把输出打飞；
         2) 限"累加总量"：把 pid->output 限在 output_limit 内，防止积分饱和。 */
    if(pid->integral_limit > 0.0f)
    {
        delta = clamp_float(delta, -pid->integral_limit, pid->integral_limit);
    }

    // 输出累加
    pid->output += delta;

    // 累加值限幅，等效于位置式 PID 的积分限幅 + 输出限幅
    if(pid->output_limit > 0.0f)
    {
        pid->output = clamp_float(pid->output, -pid->output_limit, pid->output_limit);
    }

    // 更新历史误差与状态量
    pid->prev_error = e1;
    pid->last_error = e2;
    pid->error = e2;
    pid->target = target;
    pid->actual = actual;

    return pid->output;
}

/**
 * @brief 初始化控制模块。
 */
void Control_Init(void)
{
    Pid_Init(&g_servo_pid, PID_POSITION);
    g_running = 0;
    g_turning_center = IMAGE_CENTER_X; // 将转向中心设为图像左右中心，保证一开始前轮朝前
    #if MOTOR_CLOSED_LOOP
    Pid_Init(&g_motorL_pid, PID_INCREMENTAL);
    Pid_Init(&g_motorR_pid, PID_INCREMENTAL);
    g_motorL_pid.output_limit = 4000.0f; // 舵机闭环输出限幅，影响加减速能力
    g_motorR_pid.output_limit = 4000.0f; // 舵机闭环输出限幅，影响加减速能力
    g_motorL_pid.integral_limit = 300.0f; // 增量式 PID 的单次增量限幅，防止输出突变（见问答题 1）
    g_motorR_pid.integral_limit = 300.0f; // 增量式 PID 的单次增量限幅，防止输出突变（见问答题 1）
    #else
    // g_motor_base_pwm = 2000;
    #endif
}

/**
 * @brief 5ms 周期控制中断回调函数
 */
void Control_Update5ms(void)
{
    // 编程题 1：非阻塞检测发车按键（累计按住 2s 发车），与 START 指令互不干扰
    Control_CheckStartKey();
    // 读取编码器
    Encoder_Update();
    // 传入转向中心，计算舵机 PID 控制舵机
    Control_ServoByTurningCenter(g_turning_center);
    printf("center: %d, servo: %d\n", g_turning_center, (int)g_servo_pid.output);
    // 运行标志为 0 时停车
    if(g_running == 0)
    {
        Motor_Stop();
        return;
    }
    // 控制电机
    #if MOTOR_CLOSED_LOOP
    Control_MotorBySpeed(g_motor_base_speed, g_servo_pid.output);
    #else
    Control_MotorByPwm(g_motor_base_pwm, g_servo_pid.output);
    #endif
}

/**
 * @brief 设置车辆运行状态。
 * @param running 非 0 表示发车，0 表示停车。
 */
void Control_SetRunState(int running)
{
    g_running = (running != 0) ? 1 : 0;
}

/**
 * @brief 获取车辆运行状态。
 * @retval 非 0 表示运行，0 表示停止。
 */
int Control_IsRunning(void)
{
    return g_running;
}

/**
 * @brief 设置转向中心。
 * @param center 图像处理得到的转向中心。
 */
void Control_SetTurningCenter(int center)
{
    g_turning_center = center;
}

/**
 * @brief 设置电机开环控制基础 PWM 值
 * @param pwm 基础 PWM 值
 */
void Control_SetMotorBasePwm(int pwm)
{
    #if !MOTOR_CLOSED_LOOP
    g_motor_base_pwm = pwm;
    #endif
}

/**
 * @brief 设置电机闭环控制基础速度
 * @param speed 基础速度
 */
void Control_SetMotorBaseSpeed(int speed)
{
    #if MOTOR_CLOSED_LOOP
    g_motor_base_speed = speed;
    #endif
}

/**
 * @brief 设置电机差速因子（编程题 3）。
 * @param diff_k 差速因子。
 * @note  必须是非 static 的全局函数，否则只有本文件可见，外部（指令解析处）链不到；
 *        同时这里做限幅，防止上位机误发一个很大的值把车"调飞"。
 */
void Control_SetMotorDiff(int diff_k)
{
    if(diff_k > MOTOR_DIFF_MAX)
    {
        diff_k = MOTOR_DIFF_MAX;
    }
    if(diff_k < -MOTOR_DIFF_MAX)
    {
        diff_k = -MOTOR_DIFF_MAX;
    }
    g_motor_diff_k = diff_k;
    printf("[Control] motor_diff_k = %d\n", g_motor_diff_k);
}

/**
 * @brief 获取当前转向中心。
 * @retval 当前转向中心。
 */
int Control_GetTurningCenter(void)
{
    return g_turning_center;
}

/**
 * @brief 设置舵机 PID 参数。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 */
void Control_SetServoPid(float kp, float ki, float kd)
{
    g_servo_pid.kp = kp;
    g_servo_pid.ki = ki;
    g_servo_pid.kd = kd;
}

/**
 * @brief 设置电机 PID 参数。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 */
void Control_SetMotorPid(float kp, float ki, float kd)
{
    #if MOTOR_CLOSED_LOOP
    g_motorL_pid.kp = kp;
    g_motorL_pid.ki = ki;
    g_motorL_pid.kd = kd;

    g_motorR_pid.kp = kp;
    g_motorR_pid.ki = ki;
    g_motorR_pid.kd = kd;
    #endif
}

/**
 * @brief 根据转向中心计算并设置舵机输出。
 * @param center 图像处理得到的转向中心。
 */
void Control_ServoByTurningCenter(int center)
{
    float output = Pid_CalcPosition(&g_servo_pid, (float)center, (float)IMAGE_CENTER_X);
    // 舵机输出值 = 舵机中值 - 舵机 PID 输出值
    int servo_pulse = SERVO_MID_VALUE - (int)output;
    // 限幅
    if(servo_pulse > SERVO_MAX_VALUE)
    {
        servo_pulse = SERVO_MAX_VALUE;
    }
    if(servo_pulse < SERVO_MIN_VALUE)
    {
        servo_pulse = SERVO_MIN_VALUE;
    }
    // 调用舵机接口
    Servo_SetPulse((unsigned int)servo_pulse);
}

/**
 * @brief 电机开环控制
 * @param base_pwm PWM 基础值
 * @param servo_output 舵机输出值（用于计算差速）
 */
void Control_MotorByPwm(int base_pwm, float servo_output)
{
    #if !MOTOR_CLOSED_LOOP
    int diff_num = g_motor_diff_k * ABS((int)servo_output); // 差速中间变量
    // 差速限幅
    if(diff_num > 3000)
    {
        diff_num = 3000;
    }
    else if(diff_num < 0)
    {
        diff_num = 0;
    }
    // 应用差速
    int pwmL = base_pwm;
    int pwmR = base_pwm;
    if((int)servo_output < 0) // 左转
    {
        pwmR = (int)(pwmR * (10000 + diff_num) / 10000);
        pwmL = (int)(pwmL * (10000 - diff_num) / 10000);
    }
    else if((int)servo_output > 0) // 右转
    {
        pwmL = (int)(pwmL * (10000 + diff_num) / 10000);
        pwmR = (int)(pwmR * (10000 - diff_num) / 10000);
    }
    if(g_running)
    {
        Motor_SetPwm(pwmL, -pwmR); // 如果有电机转反了，就反转其符号
    }
    else // 停车时停止 PWM 输出，小车会惯性滑行。想要快速开环停车可以短暂输出一个负的 PWM 值
    {
        Motor_SetPwm(0, 0);
    }
    #endif
}

/**
 * @brief 电机闭环控制
 * @param target_speed 目标速度
 * @param servo_output 舵机输出值（用于计算差速）
 */
void Control_MotorBySpeed(int target_speed, float servo_output)
{
    #if MOTOR_CLOSED_LOOP
    /* 1. 用舵机输出计算左右轮的差速，思路与开环 Control_MotorByPwm 完全一致，
          保证开环 / 闭环两种模式下小车的转向手感一致 */
    int diff_num = g_motor_diff_k * ABS((int)servo_output); // 差速中间变量
    if(diff_num > 3000)
    {
        diff_num = 3000;
    }
    else if(diff_num < 0)
    {
        diff_num = 0;
    }

    int target_l = target_speed; // 左轮目标速度
    int target_r = target_speed; // 右轮目标速度
    if((int)servo_output < 0) // 左转：右轮加速、左轮减速
    {
        target_r = (int)(target_r * (10000 + diff_num) / 10000);
        target_l = (int)(target_l * (10000 - diff_num) / 10000);
    }
    else if((int)servo_output > 0) // 右转：左轮加速、右轮减速
    {
        target_l = (int)(target_l * (10000 + diff_num) / 10000);
        target_r = (int)(target_r * (10000 - diff_num) / 10000);
    }

    /* 2. 左右轮各用一个增量式 PID 闭环：
          目标值 = 上面的差速后目标速度，反馈值 = 编码器实测速度
          增量式 PID 返回的是"累加后的输出"，正好可以直接当 PWM 用 */
    const int speed_l = Encoder_GetLeftSpeed();
    const int speed_r = Encoder_GetRightSpeed();

    const int pwm_l = (int)Pid_CalcIncremental(&g_motorL_pid, (float)target_l, (float)speed_l);
    const int pwm_r = (int)Pid_CalcIncremental(&g_motorR_pid, (float)target_r, (float)speed_r);

    if(g_running)
    {
        Motor_SetPwm(pwm_l, -pwm_r); // 与开环一致：右轮取反（若有电机转反了就反转其符号）
    }
    else
    {
        Motor_SetPwm(0, 0);
    }
    #endif
}

/**
 * @brief 处理控制参数类指令。
 * @param command 解析后的上位机指令。
 * @retval 1 已处理。
 * @retval 0 不是本模块负责的指令。
 */
int Control_HandleCommand(const ProtocolCommand *command)
{
    if(command == NULL)
    {
        return 0;
    }

    if(strcmp(command->type, "SERVO_PID") == 0 && command->param_count >= 3) // 舵机 PID 设置指令
    {
        Control_SetServoPid((float)atof(command->params[0]), (float)atof(command->params[1]), (float)atof(command->params[2]));
        return 1;
    }

    if(strcmp(command->type, "MOTOR_BASE") == 0 && command->param_count >= 1) // 电机设置指令
    {
        #if MOTOR_CLOSED_LOOP
        g_motor_base_speed = atoi(command->params[0]);  // 闭环则设置基础目标速度
        #else
        g_motor_base_pwm = atoi(command->params[0]); // 开环则设置基础 PWM 值
        #endif
        return 1;
    }

    /* 编程题 3：上位机用 `MOTOR_DIFF, 30` 设定差速因子，
       这条分支把"指令 -> Control_SetMotorDiff"的链路接起来。
       注意 Control_SetMotorDiff 必须是全局函数（非 static），否则链接不到。 */
    if(strcmp(command->type, "MOTOR_DIFF") == 0 && command->param_count >= 1)
    {
        Control_SetMotorDiff(atoi(command->params[0]));
        return 1;
    }

    return 0;
}