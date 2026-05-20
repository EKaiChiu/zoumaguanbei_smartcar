#ifndef ELEMENT_HPP
#define ELEMENT_HPP

#include "zf_common_headfile.hpp"

typedef enum
{
    ELEMENT_NORMAL = 0,
    ELEMENT_STRAIGHT,
    ELEMENT_CROSS,
    ELEMENT_RING_LEFT,
    ELEMENT_RING_RIGHT,
    ELEMENT_OBSTACLE,
    ELEMENT_ZEBRA
} ElementType;

void element_process(void);
void display_gray_with_pid(uint8 *image);

ElementType element_get_type(void);

int element_is_straight(void);
int element_is_cross(void);
int element_is_obstacle(void);

#endif