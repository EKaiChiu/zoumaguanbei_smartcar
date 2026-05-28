#include "menu.hpp"
#include "config.hpp"
#include "Key.hpp"
#include "pid.hpp"
#include "trackline.hpp"
#include "element.hpp"
#include "circle.hpp"
#include "crossing.hpp"
#include "zf_common_headfile.hpp"

/*
    按键逻辑：

    未发车参数菜单：
        KEY1：向上选择参数
        KEY2：向下选择参数
        KEY3：进入编辑模式 / 确认退出编辑模式
        KEY4：发车

    参数编辑模式：
        KEY1：当前参数减小
        KEY2：当前参数增大
        KEY3：确认并退出编辑模式
        KEY4：发车

    发车后：
        KEY4：停车，回到参数菜单
*/

typedef enum
{
    MENU_RUN_SPEED = 0,
    MENU_AIM_Y,

    MENU_SPEED_KP,
    MENU_SPEED_KI,
    MENU_SPEED_KD,

    MENU_TRACK_KP,
    MENU_TRACK_KI,
    MENU_TRACK_KD,

    MENU_DIFF_GAIN,
    MENU_DIFF_LIMIT,

    MENU_ITEM_COUNT
} MenuItem;

static int menu_enabled = 1;
static int menu_editing = 0;
static int menu_index = MENU_RUN_SPEED;
static int car_started = 0;

/*
函数名称：static int limit_int_local(int value, int min_value, int max_value)
功能说明：限制整数变量范围
参数说明：
    value：输入值
    min_value：最小值
    max_value：最大值
函数返回：限幅后的整数
修改时间：2026年5月28日
备注：
example：  limit_int_local(value, 0, 800);
 */
static int limit_int_local(int value, int min_value, int max_value)
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
函数名称：static double limit_double_local(double value, double min_value, double max_value)
功能说明：限制浮点变量范围
参数说明：
    value：输入值
    min_value：最小值
    max_value：最大值
函数返回：限幅后的浮点数
修改时间：2026年5月28日
备注：
example：  limit_double_local(value, 0.0, 10.0);
 */
static double limit_double_local(double value, double min_value, double max_value)
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
函数名称：static int double_to_int_100(double value)
功能说明：将浮点数放大100倍后转成整数显示
参数说明：
    value：输入浮点数
函数返回：放大100倍后的整数
修改时间：2026年5月28日
备注：
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
功能说明：将浮点数放大1000倍后转成整数显示
参数说明：
    value：输入浮点数
函数返回：放大1000倍后的整数
修改时间：2026年5月28日
备注：
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
函数名称：static void menu_limit_config(void)
功能说明：限制菜单参数范围，防止参数调到异常值
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_limit_config();
 */
static void menu_limit_config(void)
{
    car_config.run_speed = limit_int_local(car_config.run_speed, 0, 800);
    car_config.aim_y = limit_int_local(car_config.aim_y, 20, 115);

    car_config.speed_kp = limit_double_local(car_config.speed_kp, 0.0, 100.0);
    car_config.speed_ki = limit_double_local(car_config.speed_ki, 0.0, 5.0);
    car_config.speed_kd = limit_double_local(car_config.speed_kd, 0.0, 100.0);

    car_config.track_kp = limit_double_local(car_config.track_kp, 0.0, 20.0);
    car_config.track_ki = limit_double_local(car_config.track_ki, 0.0, 5.0);
    car_config.track_kd = limit_double_local(car_config.track_kd, 0.0, 20.0);

    car_config.diff_gain = limit_double_local(car_config.diff_gain, 0.1, 20.0);
    car_config.diff_speed_lim = limit_double_local(car_config.diff_speed_lim, 50.0, 1000.0);
}

/*
函数名称：static void menu_apply_speed_pid(void)
功能说明：将菜单中的速度PID参数应用到左右轮速度PID
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_apply_speed_pid();
 */
static void menu_apply_speed_pid(void)
{
    pid_init(&RSPID,
             car_config.speed_kp,
             car_config.speed_ki,
             car_config.speed_kd);

    pid_init(&LSPID,
             car_config.speed_kp,
             car_config.speed_ki,
             car_config.speed_kd);
}

/*
函数名称：static void menu_apply_track_pid(void)
功能说明：将菜单中的巡线PID参数应用到巡线控制
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_apply_track_pid();
 */
static void menu_apply_track_pid(void)
{
    trackline_init(car_config.track_kp,
                   car_config.track_ki,
                   car_config.track_kd);
}

/*
函数名称：static double menu_get_step(void)
功能说明：获取当前菜单项调节步进
参数说明：无
函数返回：当前参数对应的步进值
修改时间：2026年5月28日
备注：
example：  menu_get_step();
 */
static double menu_get_step(void)
{
    if(menu_index == MENU_RUN_SPEED)
    {
        return 20.0;
    }
    else if(menu_index == MENU_AIM_Y)
    {
        return 1.0;
    }
    else if(menu_index == MENU_SPEED_KP)
    {
        return 0.5;
    }
    else if(menu_index == MENU_SPEED_KI)
    {
        return 0.001;
    }
    else if(menu_index == MENU_SPEED_KD)
    {
        return 0.05;
    }
    else if(menu_index == MENU_TRACK_KP)
    {
        return 0.1;
    }
    else if(menu_index == MENU_TRACK_KI)
    {
        return 0.001;
    }
    else if(menu_index == MENU_TRACK_KD)
    {
        return 0.05;
    }
    else if(menu_index == MENU_DIFF_GAIN)
    {
        return 0.1;
    }
    else if(menu_index == MENU_DIFF_LIMIT)
    {
        return 20.0;
    }

    return 1.0;
}

/*
函数名称：static void menu_value_change(int dir)
功能说明：根据方向修改当前选中的参数
参数说明：
    dir：修改方向，1表示增加，-1表示减少
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_value_change(1);
 */
static void menu_value_change(int dir)
{
    double step = menu_get_step();

    if(menu_index == MENU_RUN_SPEED)
    {
        car_config.run_speed += (int)(dir * step);
    }
    else if(menu_index == MENU_AIM_Y)
    {
        car_config.aim_y += (int)(dir * step);
    }
    else if(menu_index == MENU_SPEED_KP)
    {
        car_config.speed_kp += dir * step;
    }
    else if(menu_index == MENU_SPEED_KI)
    {
        car_config.speed_ki += dir * step;
    }
    else if(menu_index == MENU_SPEED_KD)
    {
        car_config.speed_kd += dir * step;
    }
    else if(menu_index == MENU_TRACK_KP)
    {
        car_config.track_kp += dir * step;
    }
    else if(menu_index == MENU_TRACK_KI)
    {
        car_config.track_ki += dir * step;
    }
    else if(menu_index == MENU_TRACK_KD)
    {
        car_config.track_kd += dir * step;
    }
    else if(menu_index == MENU_DIFF_GAIN)
    {
        car_config.diff_gain += dir * step;
    }
    else if(menu_index == MENU_DIFF_LIMIT)
    {
        car_config.diff_speed_lim += dir * step;
    }

    menu_limit_config();

    if(menu_index == MENU_SPEED_KP ||
       menu_index == MENU_SPEED_KI ||
       menu_index == MENU_SPEED_KD)
    {
        menu_apply_speed_pid();
    }

    if(menu_index == MENU_TRACK_KP ||
       menu_index == MENU_TRACK_KI ||
       menu_index == MENU_TRACK_KD)
    {
        menu_apply_track_pid();
    }
}

/*
函数名称：static const char *menu_get_name(int index)
功能说明：获取菜单项名称
参数说明：
    index：菜单项编号
函数返回：菜单项名称字符串
修改时间：2026年5月28日
备注：
example：  menu_get_name(menu_index);
 */
static const char *menu_get_name(int index)
{
    if(index == MENU_RUN_SPEED)
    {
        return "RUN";
    }
    else if(index == MENU_AIM_Y)
    {
        return "AIM";
    }
    else if(index == MENU_SPEED_KP)
    {
        return "SKP";
    }
    else if(index == MENU_SPEED_KI)
    {
        return "SKI";
    }
    else if(index == MENU_SPEED_KD)
    {
        return "SKD";
    }
    else if(index == MENU_TRACK_KP)
    {
        return "TKP";
    }
    else if(index == MENU_TRACK_KI)
    {
        return "TKI";
    }
    else if(index == MENU_TRACK_KD)
    {
        return "TKD";
    }
    else if(index == MENU_DIFF_GAIN)
    {
        return "DGAIN";
    }
    else if(index == MENU_DIFF_LIMIT)
    {
        return "DLIM";
    }

    return "NONE";
}

/*
函数名称：static int menu_get_show_value(int index)
功能说明：获取菜单项显示值
参数说明：
    index：菜单项编号
函数返回：显示值
修改时间：2026年5月28日
备注：
    SKP/SKD/TKP/TKD/DGAIN 显示值 = 实际值 * 100
    SKI/TKI 显示值 = 实际值 * 1000
example：  menu_get_show_value(menu_index);
 */
static int menu_get_show_value(int index)
{
    if(index == MENU_RUN_SPEED)
    {
        return car_config.run_speed;
    }
    else if(index == MENU_AIM_Y)
    {
        return car_config.aim_y;
    }
    else if(index == MENU_SPEED_KP)
    {
        return double_to_int_100(car_config.speed_kp);
    }
    else if(index == MENU_SPEED_KI)
    {
        return double_to_int_1000(car_config.speed_ki);
    }
    else if(index == MENU_SPEED_KD)
    {
        return double_to_int_100(car_config.speed_kd);
    }
    else if(index == MENU_TRACK_KP)
    {
        return double_to_int_100(car_config.track_kp);
    }
    else if(index == MENU_TRACK_KI)
    {
        return double_to_int_1000(car_config.track_ki);
    }
    else if(index == MENU_TRACK_KD)
    {
        return double_to_int_100(car_config.track_kd);
    }
    else if(index == MENU_DIFF_GAIN)
    {
        return double_to_int_100(car_config.diff_gain);
    }
    else if(index == MENU_DIFF_LIMIT)
    {
        return (int)car_config.diff_speed_lim;
    }

    return 0;
}

/*
函数名称：static int menu_get_show_step(int index)
功能说明：获取菜单项显示用步进值
参数说明：
    index：菜单项编号
函数返回：显示用步进值
修改时间：2026年5月28日
备注：
example：  menu_get_show_step(menu_index);
 */
static int menu_get_show_step(int index)
{
    double step = menu_get_step();

    if(index == MENU_SPEED_KI || index == MENU_TRACK_KI)
    {
        return double_to_int_1000(step);
    }

    if(index == MENU_RUN_SPEED ||
       index == MENU_AIM_Y ||
       index == MENU_DIFF_LIMIT)
    {
        return (int)step;
    }

    return double_to_int_100(step);
}

/*
函数名称：static void menu_show_mark(int x, int y, int selected, int editing)
功能说明：显示菜单项前方状态标志
参数说明：
    x：横坐标
    y：纵坐标
    selected：是否选中当前项
    editing：是否正在编辑当前项
函数返回：无
修改时间：2026年5月28日
备注：
    Y：选中但未编辑
    R：正在编辑
example：  menu_show_mark(0, y, 1, menu_editing);
 */
static void menu_show_mark(int x, int y, int selected, int editing)
{
    if(!selected)
    {
        ips200.show_string(x, y, " ");
        return;
    }

    if(editing)
    {
        ips200.show_string(x, y, "R");
    }
    else
    {
        ips200.show_string(x, y, "Y");
    }
}

/*
函数名称：void menu_init(void)
功能说明：初始化菜单状态
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_init();
 */
void menu_init(void)
{
    menu_enabled = 1;
    menu_editing = 0;
    menu_index = MENU_RUN_SPEED;
    car_started = 0;

    menu_limit_config();
    menu_apply_speed_pid();
    menu_apply_track_pid();
}

/*
函数名称：void menu_process(void)
功能说明：菜单按键处理函数
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
    KEY1/KEY2：
        未发车时选择或调节参数；
        发车后不处理参数调节。
    KEY3：
        未发车时进入/退出编辑模式。
    KEY4：
        发车/停车。
example：  menu_process();
 */
void menu_process(void)
{
    Key_Process();

    int key = Key_GetValueOnce();

    if(key == KEY_NONE)
    {
        return;
    }

    if(key == KEY_4)
    {
        if(car_started)
        {
            menu_car_stop();
        }
        else
        {
            menu_editing = 0;
            car_started = 1;
        }

        return;
    }

    /*
        车动的时候不允许调配置。
        发车后 KEY1/KEY2/KEY3 不再修改参数。
     */
    if(car_started)
    {
        return;
    }

    if(key == KEY_3)
    {
        menu_editing = !menu_editing;
        return;
    }

    if(menu_editing)
    {
        if(key == KEY_1)
        {
            menu_value_change(-1);
        }
        else if(key == KEY_2)
        {
            menu_value_change(1);
        }
    }
    else
    {
        if(key == KEY_1)
        {
            menu_index--;

            if(menu_index < 0)
            {
                menu_index = MENU_ITEM_COUNT - 1;
            }
        }
        else if(key == KEY_2)
        {
            menu_index++;

            if(menu_index >= MENU_ITEM_COUNT)
            {
                menu_index = 0;
            }
        }
    }
}

/*
函数名称：int menu_is_enabled(void)
功能说明：获取菜单是否显示
参数说明：无
函数返回：
    1：显示菜单
    0：不显示菜单
修改时间：2026年5月28日
备注：
example：  menu_is_enabled();
 */
int menu_is_enabled(void)
{
    return menu_enabled;
}

/*
函数名称：int menu_is_editing(void)
功能说明：获取当前是否处于编辑模式
参数说明：无
函数返回：
    1：正在编辑参数
    0：正在选择参数
修改时间：2026年5月28日
备注：
example：  menu_is_editing();
 */
int menu_is_editing(void)
{
    return menu_editing;
}

/*
函数名称：int menu_car_is_started(void)
功能说明：获取当前发车状态
参数说明：无
函数返回：
    1：已经发车
    0：未发车
修改时间：2026年5月28日
备注：
example：  menu_car_is_started();
 */
int menu_car_is_started(void)
{
    return car_started;
}

/*
函数名称：void menu_car_stop(void)
功能说明：强制停车并回到参数菜单
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
    停车后允许重新选择参数、修改参数，然后再次发车。
example：  menu_car_stop();
 */
void menu_car_stop(void)
{
    car_started = 0;
    menu_enabled = 1;
    menu_editing = 0;

    pid_speed_set_target(0, 0);

    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);
}

/*
函数名称：void menu_show(void)
功能说明：未发车时显示参数调节菜单
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
example：  menu_show();
 */
void menu_show(void)
{
    if(!menu_enabled)
    {
        return;
    }

    ips200.show_string(0, 0, "MENU");

    ips200.show_string(50, 0, "RUN:");
    ips200.show_int(90, 0, car_started, 1);

    ips200.show_string(110, 0, "EDIT:");
    ips200.show_int(155, 0, menu_editing, 1);

    int page_start = 0;

    if(menu_index >= 6)
    {
        page_start = menu_index - 5;
    }

    for(int i = 0; i < 6; i++)
    {
        int item = page_start + i;

        if(item >= MENU_ITEM_COUNT)
        {
            break;
        }

        int y = 25 + i * 20;

        menu_show_mark(0, y, item == menu_index, menu_editing);

        ips200.show_string(15, y, menu_get_name(item));

        ips200.show_string(75, y, ":");
        ips200.show_int(85, y, menu_get_show_value(item), 5);

        ips200.show_string(135, y, "S:");
        ips200.show_int(155, y, menu_get_show_step(item), 4);
    }

    ips200.show_string(0, 150, "K1 UP/- K2 DN/+");
    ips200.show_string(0, 170, "K3 OK  K4 START");
    ips200.show_string(0, 190, "x100:KP KD DG");
    ips200.show_string(0, 210, "x1000:KI");
}

/*
函数名称：void menu_show_run_state(void)
功能说明：发车后显示道路元素状态，不参与参数调节
参数说明：无
函数返回：无
修改时间：2026年5月28日
备注：
    发车后只显示状态，不允许 KEY1/KEY2/KEY3 修改配置。
example：  menu_show_run_state();
 */
void menu_show_run_state(void)
{
    ips200.show_string(0, 180, "ELEM:");
    ips200.show_int(45, 180, (int)element_get_type(), 2);

    ips200.show_string(80, 180, "STR:");
    ips200.show_int(120, 180, element_is_straight(), 1);

    ips200.show_string(0, 200, "CRS:");
    ips200.show_int(40, 200, element_is_cross(), 1);
    
    ips200.show_string(80, 200, "CF:");
    ips200.show_int(110, 200, Cross_Flaga, 1);

    ips200.show_string(0, 220, "CIR:");
    ips200.show_int(40, 220, circle_island_is_found(), 1);

    ips200.show_string(80, 220, "IS:");
    ips200.show_int(110, 220, Island_State, 2);

    ips200.show_string(135, 220, "L:");
    ips200.show_int(155, 220, Left_Island_Flag, 1);

    ips200.show_string(170, 220, "R:");
    ips200.show_int(190, 220, Right_Island_Flag, 1);
}