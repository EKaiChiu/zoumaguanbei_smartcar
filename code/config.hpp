#ifndef CONFIG_HPP
#define CONFIG_HPP

typedef enum
{
    CAR_MODE_RACE = 0,       // 比赛模式：KEY4 一键发车，不显示调参菜单
    CAR_MODE_TUNE,           // 调车模式：当前菜单调参模式
    CAR_MODE_IMAGE_TEST      // 图像测试模式：只看图像，电机不动
} CarWorkMode;

typedef struct
{
    CarWorkMode work_mode;

    int run_speed;
    int aim_y;

    double speed_kp;
    double speed_ki;
    double speed_kd;

    double track_kp;
    double track_ki;
    double track_kd;

    double diff_gain;
    double diff_speed_lim;
} CarConfig;

extern CarConfig car_config;

void config_init(void);

int config_is_race_mode(void);
int config_is_tune_mode(void);
int config_is_image_test_mode(void);
int config_motor_enable(void);

#endif