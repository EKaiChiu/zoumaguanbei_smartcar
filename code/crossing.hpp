#ifndef CROSSING_HPP
#define CROSSING_HPP

#include "image.hpp"

void Cross_Detect(void);

void Find_Up_Point(int start, int end);
void Find_Down_Point(int start, int end);

void Left_Add_Line(int x1, int y1, int x2, int y2);
void Right_Add_Line(int x1, int y1, int x2, int y2);

void Lengthen_Left_Boundry(int start, int end);
void Lengthen_Right_Boundry(int start, int end);

extern volatile int Cross_Flaga;

extern volatile int Left_Down_Find;
extern volatile int Left_Up_Find;
extern volatile int Right_Down_Find;
extern volatile int Right_Up_Find;

#endif