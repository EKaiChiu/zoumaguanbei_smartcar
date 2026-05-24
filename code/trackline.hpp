#ifndef TRACKLINE_HPP
#define TRACKLINE_HPP

#include "zf_common_headfile.hpp"

void trackline_init(float kp, float ki, float kd);

void trackline_refresh_wheel_targets(int base_speed, int aim_y);

int trackline_wheel_target_right(void);
int trackline_wheel_target_left(void);

float trackline_get_error(void);
float trackline_get_turn_output(void);
extern  int wheel_target_left;
extern  int wheel_target_right;
extern int diff;

#endif