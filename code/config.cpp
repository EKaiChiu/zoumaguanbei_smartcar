#include "config.hpp"

CarConfig car_config;

void config_init(void)
{
    car_config.run_speed = 400;

    car_config.speed_kp = 10.0;
    car_config.speed_ki = 0.01;
    car_config.speed_kd = 0.0;

    car_config.track_kp = 2.0;
    car_config.track_ki = 0.0;
    car_config.track_kd = 0.5;

    // 差速放大系数
    car_config.diff_gain = 5.0f;

    //最大差速限幅
    car_config.diff_speed_lim = 800.0;

}