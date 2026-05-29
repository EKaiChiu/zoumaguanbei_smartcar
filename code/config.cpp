#include "config.hpp"

CarConfig car_config;

/*
函数名称：void config_init(void)
功能说明：初始化车辆运行配置
参数说明：无
函数返回：无
修改时间：2026年5月29日
备注：
    work_mode 在这里固定。
    改完模式后需要重新编译下载。
example：  config_init();
 */
void config_init(void)
{
    /*
        在这里选择当前程序模式：

        CAR_MODE_RACE       比赛模式
        CAR_MODE_TUNE       调车模式
        CAR_MODE_IMAGE_TEST 图像测试模式
     */
    car_config.work_mode = CAR_MODE_TUNE;

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

int config_is_race_mode(void)
{
    return car_config.work_mode == CAR_MODE_RACE;
}

int config_is_tune_mode(void)
{
    return car_config.work_mode == CAR_MODE_TUNE;
}

int config_is_image_test_mode(void)
{
    return car_config.work_mode == CAR_MODE_IMAGE_TEST;
}

int config_motor_enable(void)
{
    if(car_config.work_mode == CAR_MODE_IMAGE_TEST)
    {
        return 0;
    }

    return 1;
}