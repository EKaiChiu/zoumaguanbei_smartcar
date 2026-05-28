#include "config.hpp"
#include "zf_common_headfile.hpp"

CarConfig car_config;

/*
函数名称：void config_init(void)
功能说明：初始化智能车可调参数
参数说明：无
函数返回：无
修改时间：2026年5月25日
备注：
    所有菜单可调参数的默认值都在这里设置。
example：  config_init();
 */
void config_init(void)
{
    car_config.run_speed = 400;

    car_config.aim_y = 75;

    car_config.speed_kp = 10.0;
    car_config.speed_ki = 0.05;
    car_config.speed_kd = 0.0;

    car_config.track_kp = 2.0;
    car_config.track_ki = 0.0;
    car_config.track_kd = 0.5;

    car_config.diff_gain = 5.0;
    car_config.diff_speed_lim = 800.0;
}