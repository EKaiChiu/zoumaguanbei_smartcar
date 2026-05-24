#ifndef ELEMENT_HPP
#define ELEMENT_HPP

typedef enum
{
    ELEMENT_NORMAL = 0,
    ELEMENT_STRAIGHT,
    ELEMENT_CROSS,
    ELEMENT_CIRCLE,
    ELEMENT_ZEBRA,
    ELEMENT_OBSTACLE
} ElementType;

void element_process(void);

ElementType element_get_type(void);

int element_is_straight(void);
int element_is_cross(void);
int element_is_circle(void);
int element_is_zebra(void);
int element_is_obstacle(void);

#endif