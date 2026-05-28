#ifndef CONFIG_HPP
#define CONFIG_HPP

typedef struct
{
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

#endif