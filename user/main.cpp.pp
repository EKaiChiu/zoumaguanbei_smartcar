#include "zf_common_headfile.hpp"
#include "../code/pid.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <csignal>
#include <chrono>
#include <sys/select.h>
#include <unistd.h>
#include <cctype>

#define DEFAULT_KP              10.0f
#define DEFAULT_KI              0.0f
#define DEFAULT_KD              0.0f

#define MAX_TARGET_SPEED        700
#define MIN_TARGET_SPEED        0

#define CONTROL_PERIOD_MS       20
#define PRINT_INTERVAL_MS       100

#define PID_SATURATION_VALUE    760.0f

static volatile int program_running = 1;

static int target_speed_right = 0;
static int target_speed_left = 0;

static float current_kp = DEFAULT_KP;
static float current_ki = DEFAULT_KI;
static float current_kd = DEFAULT_KD;

extern pid_class RSPID;
extern pid_class LSPID;

/*
函数名称：PidTestState
功能说明：自动 PID 测试状态结构体
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    用于统计某一组 kp、ki、kd 在指定速度下的平均误差、最大误差、输出饱和次数和实际速度范围。
example：
 */
typedef struct
{
    int active;

    int speed;
    int duration_ms;
    int warmup_ms;

    std::chrono::steady_clock::time_point start_time;

    int sample_count;

    long long sum_abs_err_r;
    long long sum_abs_err_l;

    int max_abs_err_r;
    int max_abs_err_l;

    int saturation_count_r;
    int saturation_count_l;

    int max_actual_r;
    int max_actual_l;

    int min_actual_r;
    int min_actual_l;
} PidTestState;

static PidTestState test_state;

/*
函数名称：static int abs_int_local(int value)
功能说明：求整数绝对值
参数说明：
    value：需要求绝对值的整数
函数返回：输入值的绝对值
修改时间：2026年5月23日
备    注：
example：  abs_int_local(value);
 */
static int abs_int_local(int value)
{
    if(value >= 0)
    {
        return value;
    }

    return -value;
}

/*
函数名称：static float abs_float_local(float value)
功能说明：求浮点数绝对值
参数说明：
    value：需要求绝对值的浮点数
函数返回：输入值的绝对值
修改时间：2026年5月23日
备    注：
example：  abs_float_local(value);
 */
static float abs_float_local(float value)
{
    if(value >= 0.0f)
    {
        return value;
    }

    return -value;
}

/*
函数名称：static int limit_int(int value, int min_value, int max_value)
功能说明：限制整数变量的取值范围，防止数值超过指定上下限
参数说明：
    value：需要限幅的数值
    min_value：允许的最小值
    max_value：允许的最大值
函数返回：限幅后的整数值
修改时间：2026年5月23日
备    注：
example：  limit_int(speed, 0, 700);
 */
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

/*
函数名称：static std::string to_upper_string(std::string text)
功能说明：将字符串转换为大写，便于解析终端命令
参数说明：
    text：输入字符串
函数返回：转换为大写后的字符串
修改时间：2026年5月23日
备    注：
example：  to_upper_string(command);
 */
static std::string to_upper_string(std::string text)
{
    for(size_t i = 0; i < text.size(); i++)
    {
        text[i] = (char)std::toupper((unsigned char)text[i]);
    }

    return text;
}

/*
函数名称：static int stdin_has_line(void)
功能说明：判断终端是否有输入，避免 getline 阻塞主循环
参数说明：无
函数返回：
    1：终端有输入
    0：终端没有输入
修改时间：2026年5月23日
备    注：
example：  stdin_has_line();
 */
static int stdin_has_line(void)
{
    fd_set readfds;
    struct timeval timeout;

    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int result = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

    if(result > 0 && FD_ISSET(STDIN_FILENO, &readfds))
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static void clear_pid_state(void)
功能说明：清空左右轮 PID 内部状态
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    防止上一轮测试的积分、误差和输出影响下一轮测试。
example：  clear_pid_state();
 */
static void clear_pid_state(void)
{
    RSPID.Error = 0;
    RSPID.LastError = 0;
    RSPID.PrevError = 0;
    RSPID.Error_all = 0;
    RSPID.P_out = 0;
    RSPID.I_out = 0;
    RSPID.D_out = 0;
    RSPID.output = 0;

    LSPID.Error = 0;
    LSPID.LastError = 0;
    LSPID.PrevError = 0;
    LSPID.Error_all = 0;
    LSPID.P_out = 0;
    LSPID.I_out = 0;
    LSPID.D_out = 0;
    LSPID.output = 0;
}

/*
函数名称：static void signal_handler(int signum)
功能说明：捕获 Ctrl+C 退出信号
参数说明：
    signum：系统信号编号
函数返回：无
修改时间：2026年5月23日
备    注：
example：  signal(SIGINT, signal_handler);
 */
static void signal_handler(int signum)
{
    (void)signum;
    program_running = 0;
}

/*
函数名称：static void stop_motor_keep_pit(void)
功能说明：停车但不停止 PIT 定时器
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    自动调参期间 PIT 必须保持运行，不能反复 stop/start。
    这里只清零目标速度、PID 状态和编码器计数。
example：  stop_motor_keep_pit();
 */
static void stop_motor_keep_pit(void)
{
    target_speed_right = 0;
    target_speed_left = 0;

    pid_speed_set_target(0, 0);

    clear_pid_state();

    encoder_right = 0;
    encoder_left = 0;

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
}

/*
函数名称：static void final_stop_and_release(void)
功能说明：程序退出时真正停止速度闭环并释放 PIT
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    只有程序退出时才调用 pid_speed_stop()。
example：  final_stop_and_release();
 */
static void final_stop_and_release(void)
{
    test_state.active = 0;

    stop_motor_keep_pit();

    pid_speed_stop();
}

/*
函数名称：static void apply_pid_param(float kp, float ki, float kd)
功能说明：应用新的速度环 PID 参数，并清空 PID 内部状态
参数说明：
    kp：比例系数
    ki：积分系数
    kd：微分系数
函数返回：无
修改时间：2026年5月23日
备    注：
    不停止 PIT，只重新初始化 PID 参数。
example：  apply_pid_param(10.0f, 0.0f, 0.0f);
 */
static void apply_pid_param(float kp, float ki, float kd)
{
    current_kp = kp;
    current_ki = ki;
    current_kd = kd;

    pid_init(&RSPID, current_kp, current_ki, current_kd);
    pid_init(&LSPID, current_kp, current_ki, current_kd);

    clear_pid_state();

    pid_speed_set_target(target_speed_right, target_speed_left);

    std::cout << "OK PARAM"
              << " kp=" << current_kp
              << " ki=" << current_ki
              << " kd=" << current_kd
              << std::endl;
}

/*
函数名称：static void set_speed_target(int right, int left)
功能说明：设置左右轮目标速度
参数说明：
    right：右轮目标速度
    left：左轮目标速度
函数返回：无
修改时间：2026年5月23日
备    注：
example：  set_speed_target(200, 200);
 */
static void set_speed_target(int right, int left)
{
    target_speed_right = limit_int(right, MIN_TARGET_SPEED, MAX_TARGET_SPEED);
    target_speed_left = limit_int(left, MIN_TARGET_SPEED, MAX_TARGET_SPEED);

    clear_pid_state();

    encoder_right = 0;
    encoder_left = 0;

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();

    pid_speed_set_target(target_speed_right, target_speed_left);

    std::cout << "OK SPEED"
              << " right=" << target_speed_right
              << " left=" << target_speed_left
              << std::endl;
}

/*
函数名称：static void stop_motor_target(void)
功能说明：停车但不停止 PIT
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    TEST 之间只停车，不释放 PIT。
example：  stop_motor_target();
 */
static void stop_motor_target(void)
{
    test_state.active = 0;

    stop_motor_keep_pit();

    std::cout << "OK STOP" << std::endl;
}

/*
函数名称：static void print_status(void)
功能说明：打印当前速度环状态，供 Claude 或人工查看
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    输出格式固定为 DATA，方便 Claude 解析。
example：  print_status();
 */
static void print_status(void)
{
    int err_r = pid_get_target_right() - encoder_right;
    int err_l = pid_get_target_left() - encoder_left;

    std::cout << "DATA"
              << " kp=" << current_kp
              << " ki=" << current_ki
              << " kd=" << current_kd
              << " targetR=" << pid_get_target_right()
              << " actualR=" << encoder_right
              << " errR=" << err_r
              << " outR=" << RSPID.output
              << " targetL=" << pid_get_target_left()
              << " actualL=" << encoder_left
              << " errL=" << err_l
              << " outL=" << LSPID.output
              << std::endl;
}

/*
函数名称：static void reset_test_state(int speed, int duration_ms)
功能说明：初始化一次自动测试状态
参数说明：
    speed：本次测试目标速度
    duration_ms：本次测试持续时间，单位毫秒
函数返回：无
修改时间：2026年5月23日
备    注：
example：  reset_test_state(200, 3000);
 */
static void reset_test_state(int speed, int duration_ms)
{
    test_state.active = 1;

    test_state.speed = speed;
    test_state.duration_ms = duration_ms;

    test_state.warmup_ms = duration_ms / 4;

    if(test_state.warmup_ms > 500)
    {
        test_state.warmup_ms = 500;
    }

    if(test_state.warmup_ms < 200)
    {
        test_state.warmup_ms = 200;
    }

    test_state.start_time = std::chrono::steady_clock::now();

    test_state.sample_count = 0;

    test_state.sum_abs_err_r = 0;
    test_state.sum_abs_err_l = 0;

    test_state.max_abs_err_r = 0;
    test_state.max_abs_err_l = 0;

    test_state.saturation_count_r = 0;
    test_state.saturation_count_l = 0;

    test_state.max_actual_r = -32768;
    test_state.max_actual_l = -32768;

    test_state.min_actual_r = 32767;
    test_state.min_actual_l = 32767;
}

/*
函数名称：static void start_auto_test(float kp, float ki, float kd, int speed, int duration_ms)
功能说明：启动一组自动 PID 测试
参数说明：
    kp：比例系数
    ki：积分系数
    kd：微分系数
    speed：测试目标速度
    duration_ms：测试持续时间，单位毫秒
函数返回：无
修改时间：2026年5月23日
备    注：
    TEST 不再停止和重启 PIT。
    PIT 在 main() 中只启动一次，并保持运行。
example：  start_auto_test(10, 0, 0, 200, 3000);
 */
static void start_auto_test(float kp, float ki, float kd, int speed, int duration_ms)
{
    if(test_state.active)
    {
        std::cout << "ERR TEST_RUNNING_WAIT_RESULT" << std::endl;
        return;
    }

    speed = limit_int(speed, MIN_TARGET_SPEED, MAX_TARGET_SPEED);

    if(duration_ms < 1000)
    {
        duration_ms = 1000;
    }

    stop_motor_keep_pit();

    system_delay_ms(300);

    apply_pid_param(kp, ki, kd);

    encoder_right = 0;
    encoder_left = 0;

    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();

    system_delay_ms(100);

    target_speed_right = speed;
    target_speed_left = speed;

    pid_speed_set_target(speed, speed);

    reset_test_state(speed, duration_ms);

    std::cout << "OK TEST_START"
              << " kp=" << kp
              << " ki=" << ki
              << " kd=" << kd
              << " speed=" << speed
              << " duration_ms=" << duration_ms
              << std::endl;
}

/*
函数名称：static void update_auto_test(void)
功能说明：更新自动测试统计，并在测试结束时输出 RESULT
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    RESULT 行是 Claude 自动调参的主要依据。
example：  update_auto_test();
 */
static void update_auto_test(void)
{
    if(!test_state.active)
    {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - test_state.start_time).count();

    int err_r = test_state.speed - encoder_right;
    int err_l = test_state.speed - encoder_left;

    if(elapsed_ms >= test_state.warmup_ms)
    {
        int abs_err_r = abs_int_local(err_r);
        int abs_err_l = abs_int_local(err_l);

        test_state.sample_count++;

        test_state.sum_abs_err_r += abs_err_r;
        test_state.sum_abs_err_l += abs_err_l;

        if(abs_err_r > test_state.max_abs_err_r)
        {
            test_state.max_abs_err_r = abs_err_r;
        }

        if(abs_err_l > test_state.max_abs_err_l)
        {
            test_state.max_abs_err_l = abs_err_l;
        }

        if(encoder_right > test_state.max_actual_r)
        {
            test_state.max_actual_r = encoder_right;
        }

        if(encoder_left > test_state.max_actual_l)
        {
            test_state.max_actual_l = encoder_left;
        }

        if(encoder_right < test_state.min_actual_r)
        {
            test_state.min_actual_r = encoder_right;
        }

        if(encoder_left < test_state.min_actual_l)
        {
            test_state.min_actual_l = encoder_left;
        }

        if(abs_float_local(RSPID.output) >= PID_SATURATION_VALUE)
        {
            test_state.saturation_count_r++;
        }

        if(abs_float_local(LSPID.output) >= PID_SATURATION_VALUE)
        {
            test_state.saturation_count_l++;
        }
    }

    if(elapsed_ms >= test_state.duration_ms)
    {
        long long avg_abs_err_r = 0;
        long long avg_abs_err_l = 0;

        if(test_state.sample_count > 0)
        {
            avg_abs_err_r = test_state.sum_abs_err_r / test_state.sample_count;
            avg_abs_err_l = test_state.sum_abs_err_l / test_state.sample_count;
        }

        std::cout << "RESULT"
                  << " kp=" << current_kp
                  << " ki=" << current_ki
                  << " kd=" << current_kd
                  << " speed=" << test_state.speed
                  << " samples=" << test_state.sample_count
                  << " avgAbsErrR=" << avg_abs_err_r
                  << " avgAbsErrL=" << avg_abs_err_l
                  << " maxAbsErrR=" << test_state.max_abs_err_r
                  << " maxAbsErrL=" << test_state.max_abs_err_l
                  << " satR=" << test_state.saturation_count_r
                  << " satL=" << test_state.saturation_count_l
                  << " minActualR=" << test_state.min_actual_r
                  << " maxActualR=" << test_state.max_actual_r
                  << " minActualL=" << test_state.min_actual_l
                  << " maxActualL=" << test_state.max_actual_l
                  << std::endl;

        stop_motor_target();
    }
}

/*
函数名称：static void print_help(void)
功能说明：打印终端命令说明
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
example：  print_help();
 */
static void print_help(void)
{
    std::cout << "COMMANDS:" << std::endl;
    std::cout << "  PARAM kp ki kd              设置 PID 参数，例如 PARAM 10 0 0" << std::endl;
    std::cout << "  SPEED v                     设置左右轮同一目标速度，例如 SPEED 200" << std::endl;
    std::cout << "  SPEED r l                   分别设置右轮和左轮目标速度，例如 SPEED 200 210" << std::endl;
    std::cout << "  TEST kp ki kd speed ms      自动测试一组参数，例如 TEST 10 0 0 200 3000" << std::endl;
    std::cout << "  STOP                        停车但不停止 PIT" << std::endl;
    std::cout << "  SHOW                        立即打印状态" << std::endl;
    std::cout << "  HELP                        显示帮助" << std::endl;
    std::cout << "  QUIT                        退出程序" << std::endl;
}

/*
函数名称：static void handle_command(const std::string &line)
功能说明：解析并执行终端命令
参数说明：
    line：终端输入的一整行命令
函数返回：无
修改时间：2026年5月23日
备    注：
example：  handle_command(line);
 */
static void handle_command(const std::string &line)
{
    std::stringstream ss(line);

    std::string command;
    ss >> command;

    command = to_upper_string(command);

    if(command.size() == 0)
    {
        return;
    }

    if(command == "PARAM")
    {
        float kp;
        float ki;
        float kd;

        if(ss >> kp >> ki >> kd)
        {
            apply_pid_param(kp, ki, kd);
        }
        else
        {
            std::cout << "ERR PARAM_USAGE PARAM kp ki kd" << std::endl;
        }
    }
    else if(command == "SPEED")
    {
        int right;
        int left;

        if(ss >> right)
        {
            if(ss >> left)
            {
                set_speed_target(right, left);
            }
            else
            {
                set_speed_target(right, right);
            }
        }
        else
        {
            std::cout << "ERR SPEED_USAGE SPEED v 或 SPEED r l" << std::endl;
        }
    }
    else if(command == "TEST")
    {
        float kp;
        float ki;
        float kd;
        int speed;
        int duration_ms;

        if(ss >> kp >> ki >> kd >> speed >> duration_ms)
        {
            start_auto_test(kp, ki, kd, speed, duration_ms);
        }
        else
        {
            std::cout << "ERR TEST_USAGE TEST kp ki kd speed duration_ms" << std::endl;
        }
    }
    else if(command == "STOP")
    {
        stop_motor_target();
    }
    else if(command == "SHOW")
    {
        print_status();
    }
    else if(command == "HELP")
    {
        print_help();
    }
    else if(command == "QUIT" || command == "EXIT")
    {
        program_running = 0;
    }
    else
    {
        std::cout << "ERR UNKNOWN_COMMAND " << command << std::endl;
    }
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    signal(SIGINT, signal_handler);

    system("clear");

    test_state.active = 0;

    std::cout << "PID_AUTO_TUNE_READY" << std::endl;
    std::cout << "输入 HELP 查看命令。" << std::endl;

    target_speed_right = 0;
    target_speed_left = 0;

    /*
        程序启动时只启动一次 PIT。
        后续 TEST 不再 stop/start PIT，避免回调失效。
    */
    pid_speed_init(current_kp, current_ki, current_kd, 0, 0);
    pid_speed_start(CONTROL_PERIOD_MS);

    auto last_print_time = std::chrono::steady_clock::now();

    while(program_running)
    {
        if(stdin_has_line())
        {
            std::string line;
            std::getline(std::cin, line);
            handle_command(line);
        }

        update_auto_test();

        auto now = std::chrono::steady_clock::now();
        int print_elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time).count();

        if(print_elapsed_ms >= PRINT_INTERVAL_MS)
        {
            last_print_time = now;
            print_status();
        }

        system_delay_ms(20);
    }

    final_stop_and_release();

    std::cout << "PID_AUTO_TUNE_EXIT" << std::endl;
    std::cout << "程序退出，电机已关闭。" << std::endl;

    return 0;
}