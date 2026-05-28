#include "Display.hpp"
#include "pid.hpp"
#include "config.hpp"
#include "circle.hpp"

/*
函数名称：static int double_to_int_100(double value)
功能说明：把浮点参数放大100倍后转成整数显示
参数说明：
    value：需要显示的浮点数
函数返回：放大100倍后的整数
修改时间：2026年5月25日
备注：
    例如 diff_gain = 5.00，屏幕显示 500
example：  double_to_int_100(car_config.diff_gain);
 */
static int double_to_int_100(double value)
{
    if(value >= 0)
    {
        return (int)(value * 100.0 + 0.5);
    }

    return (int)(value * 100.0 - 0.5);
}

/*
函数名称：static int double_to_int_1000(double value)
功能说明：把浮点参数放大1000倍后转成整数显示
参数说明：
    value：需要显示的浮点数
函数返回：放大1000倍后的整数
修改时间：2026年5月25日
备注：
    例如 speed_ki = 0.050，屏幕显示 50
example：  double_to_int_1000(car_config.speed_ki);
 */
static int double_to_int_1000(double value)
{
    if(value >= 0)
    {
        return (int)(value * 1000.0 + 0.5);
    }

    return (int)(value * 1000.0 - 0.5);
}

/*
函数名称：void display_pid_data(void)
功能说明：显示左右轮目标速度、编码器反馈和PID输出
参数说明：无
函数返回：无
修改时间：2026年5月25日
备注：
    RT：右轮目标速度
    LT：左轮目标速度
    RE：右轮编码器反馈
    LE：左轮编码器反馈
    RO：右轮PID输出
    LO：左轮PID输出
example：  display_pid_data();
 */
void display_pid_data(void)
{
    ips200.show_string(0, 0, "RT:");
    ips200.show_int(25, 0, pid_get_target_right(), 4);

    ips200.show_string(80, 0, "LT:");
    ips200.show_int(105, 0, pid_get_target_left(), 4);

    ips200.show_string(0, 20, "RE:");
    ips200.show_int(25, 20, encoder_right, 4);

    ips200.show_string(80, 20, "LE:");
    ips200.show_int(105, 20, encoder_left, 4);

    ips200.show_string(0, 40, "RO:");
    ips200.show_int(25, 40, (int)RSPID.output, 5);

    ips200.show_string(80, 40, "LO:");
    ips200.show_int(105, 40, (int)LSPID.output, 5);
}

/*
函数名称：void display_config_data(void)
功能说明：显示当前运行参数和PID参数
参数说明：无
函数返回：无
修改时间：2026年5月25日
备注：
    SPD：基础速度
    AY ：前瞻行
    DIF：最大差速限幅
    DG ：差速增益，显示值 = 实际值 * 100
    SKP/SKI/SKD：速度PID参数
    TKP/TKI/TKD：巡线PID参数
example：  display_config_data();
 */
void display_config_data(void)
{
    ips200.show_string(0, 65, "SPD:");
    ips200.show_int(40, 65, car_config.run_speed, 4);

    ips200.show_string(80, 65, "AY:");
    ips200.show_int(110, 65, car_config.aim_y, 3);

    ips200.show_string(0, 85, "DIF:");
    ips200.show_int(40, 85, (int)car_config.diff_speed_lim, 4);

    ips200.show_string(80, 85, "DG:");
    ips200.show_int(110, 85, double_to_int_100(car_config.diff_gain), 4);

    ips200.show_string(0, 110, "SKP:");
    ips200.show_int(40, 110, double_to_int_100(car_config.speed_kp), 5);

    ips200.show_string(0, 130, "SKI:");
    ips200.show_int(40, 130, double_to_int_1000(car_config.speed_ki), 5);

    ips200.show_string(0, 150, "SKD:");
    ips200.show_int(40, 150, double_to_int_100(car_config.speed_kd), 5);

    ips200.show_string(80, 110, "TKP:");
    ips200.show_int(120, 110, double_to_int_100(car_config.track_kp), 5);

    ips200.show_string(80, 130, "TKI:");
    ips200.show_int(120, 130, double_to_int_1000(car_config.track_ki), 5);

    ips200.show_string(80, 150, "TKD:");
    ips200.show_int(120, 150, double_to_int_100(car_config.track_kd), 5);
}

/*
函数名称：void display_circle_data(void)
功能说明：显示当前环岛状态信息
参数说明：无
函数返回：无
修改时间：2026年5月25日
备注：
    IS：Island_State，环岛状态机状态
    LF：Left_Island_Flag，左环岛标志
    RF：Right_Island_Flag，右环岛标志
    IF：circle_island_is_found()，是否处于环岛状态
example：  display_circle_data();
 */
void display_circle_data(void)
{
    ips200.show_string(0, 180, "IS:");
    ips200.show_int(25, 180, Island_State, 2);

    ips200.show_string(55, 180, "LF:");
    ips200.show_int(85, 180, Left_Island_Flag, 1);

    ips200.show_string(105, 180, "RF:");
    ips200.show_int(135, 180, Right_Island_Flag, 1);

    ips200.show_string(0, 200, "IF:");
    ips200.show_int(25, 200, circle_island_is_found(), 1);
}

/*
函数名称：void display_gray_with_pid(uint8 *image)
功能说明：发车后的运行数据显示入口
参数说明：
    image：图像指针，当前版本不显示图像
函数返回：无
修改时间：2026年5月25日
备注：
    当前用于发车后显示运行数据，不显示菜单。
    如果要显示图像，可以打开 displayimage_gray，但会覆盖 0~120 行的数据。
example：  display_gray_with_pid(gray_image);
 */
void display_gray_with_pid(uint8 *image)
{
    (void)image;

    display_pid_data();
    display_config_data();
    display_circle_data();
}