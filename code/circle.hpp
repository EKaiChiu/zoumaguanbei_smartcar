#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "zf_common_headfile.hpp"
#include "image.hpp"

extern volatile int Island_State;
extern volatile int Left_Island_Flag;
extern volatile int Right_Island_Flag;

extern volatile int Left_Up_Guai[2];
extern volatile int Right_Up_Guai[2];

int Continuity_Change_Left(int start, int end);
int Continuity_Change_Right(int start, int end);

int Monotonicity_Change_Left(int start, int end);
int Monotonicity_Change_Right(int start, int end);

int Find_Left_Down_Point(int start, int end);
int Find_Left_Up_Point(int start, int end);
int Find_Right_Down_Point(int start, int end);
int Find_Right_Up_Point(int start, int end);

void Island_Detect(void);

void circle_set_angle(float angle);
void circle_clear(void);
int circle_island_is_found(void);

#endif