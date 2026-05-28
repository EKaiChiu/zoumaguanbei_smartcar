#ifndef KEY_HPP
#define KEY_HPP

#include "zf_common_headfile.hpp"

typedef enum
{
    KEY_NONE = 0,
    KEY_1 = 1,
    KEY_2 = 2,
    KEY_3 = 3,
    KEY_4 = 4
} KeyValue;

void Key_Process(void);
bool Key_IsPressed(void);
int Key_GetValue(void);
int Key_GetValueOnce(void);
int Key_GetPressCount(void);

#endif