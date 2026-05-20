#ifndef ELEMENT_HPP
#define ELEMENT_HPP

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
ElementType element_get_type(void);

#endif