#ifndef CONFIG_HPP
#define CONFIG_HPP

typedef struct
{
    int run_speed;
    int aim_y;

    int diff_speed_lim;
    double diff_gain;

    double speed_kp;
    double speed_ki;
    double speed_kd;

    double track_kp;
    double track_ki;
    double track_kd;
} CarConfig;

extern CarConfig car_config;

void config_init(void);

#endif