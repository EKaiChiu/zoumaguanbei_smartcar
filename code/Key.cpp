#include "Key.hpp"
#include "zf_common_headfile.hpp"

#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

#define KEY_DEBOUNCE_COUNT   3

static zf_driver_gpio key_1(KEY_1_PATH, O_RDWR);
static zf_driver_gpio key_2(KEY_2_PATH, O_RDWR);
static zf_driver_gpio key_3(KEY_3_PATH, O_RDWR);
static zf_driver_gpio key_4(KEY_4_PATH, O_RDWR);

static int press_count = 0;

static int last_raw_key = KEY_NONE;
static int stable_key = KEY_NONE;
static int debounce_count = 0;

static int last_once_key = KEY_NONE;


static int key_read_raw(void)
{
    if(key_1.get_level() == 0)
    {
        return KEY_1;
    }

    if(key_2.get_level() == 0)
    {
        return KEY_2;
    }

    if(key_3.get_level() == 0)
    {
        return KEY_3;
    }

    if(key_4.get_level() == 0)
    {
        return KEY_4;
    }

    return KEY_NONE;
}


void Key_Process(void)
{
    int raw_key = key_read_raw();

    if(raw_key == last_raw_key)
    {
        if(debounce_count < KEY_DEBOUNCE_COUNT)
        {
            debounce_count++;
        }
    }
    else
    {
        debounce_count = 0;
        last_raw_key = raw_key;
    }

    if(debounce_count >= KEY_DEBOUNCE_COUNT)
    {
        if(stable_key != raw_key)
        {
            stable_key = raw_key;

            if(stable_key != KEY_NONE)
            {
                press_count++;
            }
        }
    }
}

bool Key_IsPressed(void)
{
    return stable_key != KEY_NONE;
}


int Key_GetValue(void)
{
    return stable_key;
}


int Key_GetValueOnce(void)
{
    if(stable_key != KEY_NONE && last_once_key == KEY_NONE)
    {
        last_once_key = stable_key;
        return stable_key;
    }

    if(stable_key == KEY_NONE)
    {
        last_once_key = KEY_NONE;
    }

    return KEY_NONE;
}


int Key_GetPressCount(void)
{
    return press_count;
}