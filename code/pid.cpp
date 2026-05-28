#include "pid.hpp"


pid_class RSPID;
pid_class LSPID;


// =====================================================
// 左右轮目标速度
// =====================================================
static int target_speed_right = 0;
static int target_speed_left  = 0;


// =====================================================
// PWM 参数
// =====================================================
#define BASE_PWM_PERCENT_R   14
#define BASE_PWM_PERCENT_L   14

#define MAX_PWM_PERCENT      65


#define PID_OUTPUT_LIMIT     1000


// =====================================================
// 工具函数(整数绝对值、整数限幅、浮点数限幅)
// =====================================================
static int abs_int(int value)
{
    return value >= 0 ? value : -value;
}


static int limit_int(int value, int min_value, int max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }

    if(value > max_value)
    {
        value = max_value;
    }

    return value;
}


static float limit_float(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }

    if(value > max_value)
    {
        value = max_value;
    }

    return value;
}


// =====================================================
// PID 初始化
// =====================================================
void pid_init(pid_class *pid, float Kp, float Ki, float Kd)
{
    if(pid == NULL)
    {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target = 0;

    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->Error_all = 0;

    pid->P_out = 0;
    pid->I_out = 0;
    pid->D_out = 0;

    pid->output = 0;

    pid->integral_limit = 1000;
    pid->output_limit = PID_OUTPUT_LIMIT;
}

// =====================================================
// 速度环 PID
// =====================================================
void pid_compute(pid_class *pid, int target, int measurement)
{
    if(pid == NULL)
    {
        return;
    }

    pid->target = target;

    pid->Error = pid->target - measurement;

    pid->Error_all += pid->Error;

    pid->Error_all = limit_float(
        pid->Error_all,
        -pid->integral_limit,
        pid->integral_limit
    );

    pid->P_out = pid->Kp * pid->Error;
    pid->I_out = pid->Ki * pid->Error_all;
    pid->D_out = pid->Kd * (pid->Error - pid->LastError);

    pid->output = pid->P_out + pid->I_out + pid->D_out;

    pid->output = limit_float(
        pid->output,
        -pid->output_limit,
        pid->output_limit
    );

    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
}


// =====================================================
// 右电机底层 PWM 输出
// =====================================================
static void motor_set_pwm_R(int pwm_signed)
{
    int pwm = abs_int(pwm_signed);
    int max_pwm = drv8701e_pwm_1_info.duty_max * MAX_PWM_PERCENT / 100;

    pwm = limit_int(pwm, 0, max_pwm);

    if(pwm_signed > 0)
    {
        drv8701e_dir_1.set_level(0);
        drv8701e_pwm_1.set_duty(pwm);
    }
    else if(pwm_signed < 0)
    {
        drv8701e_dir_1.set_level(1);
        drv8701e_pwm_1.set_duty(pwm);
    }
    else
    {
        drv8701e_pwm_1.set_duty(0);
    }
}


// =====================================================
// 左电机底层 PWM 输出
// =====================================================
static void motor_set_pwm_L(int pwm_signed)
{
    int pwm = abs_int(pwm_signed);
    int max_pwm = drv8701e_pwm_2_info.duty_max * MAX_PWM_PERCENT / 100;

    pwm = limit_int(pwm, 0, max_pwm);

    if(pwm_signed > 0)
    {
        drv8701e_dir_2.set_level(0);
        drv8701e_pwm_2.set_duty(pwm);
    }
    else if(pwm_signed < 0)
    {
        drv8701e_dir_2.set_level(1);
        drv8701e_pwm_2.set_duty(pwm);
    }
    else
    {
        drv8701e_pwm_2.set_duty(0);
    }
}


// =====================================================
// 兼容旧接口
// =====================================================
void Motor_set_speed_R(float right_pwm)
{
    motor_set_pwm_R((int)right_pwm);
}


void Motor_set_speed_L(float left_pwm)
{
    motor_set_pwm_L((int)left_pwm);
}


// =====================================================
// 速度闭环定时回调
// =====================================================
static void pid_speed_callback(void)
{
    encoder_right = -encoder_dir_1.get_count();
    encoder_left  =  encoder_dir_2.get_count();

    pid_compute(&RSPID, target_speed_right, encoder_right);
    pid_compute(&LSPID, target_speed_left,  encoder_left);

    int base_pwm_R = drv8701e_pwm_1_info.duty_max * BASE_PWM_PERCENT_R / 100;
    int base_pwm_L = drv8701e_pwm_2_info.duty_max * BASE_PWM_PERCENT_L / 100;

    int cmd_R = 0;
    int cmd_L = 0;


    if(target_speed_right > 0)//前驱反馈
    {
        int ff_R = base_pwm_R * target_speed_right / 300;

        ff_R = limit_int(ff_R, base_pwm_R / 2, base_pwm_R);

        cmd_R = ff_R + (int)RSPID.output;

        if(cmd_R < 0)
        {
            cmd_R = 0;
        }
    }
    else if(target_speed_right < 0)
    {
        int ff_R = base_pwm_R * (-target_speed_right) / 300;

        ff_R = limit_int(ff_R, base_pwm_R / 2, base_pwm_R);

        cmd_R = -(ff_R - (int)RSPID.output);

        if(cmd_R > 0)
        {
            cmd_R = 0;
        }
    }
    else
    {
        cmd_R = 0;
    }

    /*
        左轮
    */
    if(target_speed_left > 0)
    {
        int ff_L = base_pwm_L * target_speed_left / 300;

        ff_L = limit_int(ff_L, base_pwm_L / 2, base_pwm_L);

        cmd_L = ff_L + (int)LSPID.output;

        if(cmd_L < 0)
        {
            cmd_L = 0;
        }
    }
    else if(target_speed_left < 0)
    {
        int ff_L = base_pwm_L * (-target_speed_left) / 300;

        ff_L = limit_int(ff_L, base_pwm_L / 2, base_pwm_L);

        cmd_L = -(ff_L - (int)LSPID.output);

        if(cmd_L > 0)
        {
            cmd_L = 0;
        }
    }
    else
    {
        cmd_L = 0;
    }

    motor_set_pwm_R(cmd_R);
    motor_set_pwm_L(cmd_L);

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
}


// =====================================================
// 速度闭环初始化
// =====================================================
void pid_speed_init(float kp, float ki, float kd, int target_right, int target_left)
{
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    pid_init(&RSPID, kp, ki, kd);
    pid_init(&LSPID, kp, ki, kd);

    target_speed_right = target_right;
    target_speed_left  = target_left;

    encoder_right = 0;
    encoder_left = 0;
}


// =====================================================
// 设置左右轮目标速度
// =====================================================
void pid_speed_set_target(int target_right, int target_left)
{
    target_speed_right = target_right;
    target_speed_left  = target_left;
}


// =====================================================
// 启动速度闭环定时器
// =====================================================
void pid_speed_start(uint32 period_ms)
{
    pit_timer.init_ms(period_ms, pid_speed_callback);
}


// =====================================================
// 停止速度闭环
// =====================================================
void pid_speed_stop(void)
{
    pit_timer.stop();

    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);

    target_speed_right = 0;
    target_speed_left = 0;

    RSPID.output = 0;
    LSPID.output = 0;

    RSPID.Error = 0;
    LSPID.Error = 0;

    RSPID.Error_all = 0;
    LSPID.Error_all = 0;

    RSPID.LastError = 0;
    LSPID.LastError = 0;

    RSPID.PrevError = 0;
    LSPID.PrevError = 0;
}


// =====================================================
// 获取目标速度
// =====================================================
int pid_get_target_right(void)
{
    return target_speed_right;
}


int pid_get_target_left(void)
{
    return target_speed_left;
}