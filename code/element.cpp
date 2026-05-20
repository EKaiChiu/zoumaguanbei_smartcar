#include "element.hpp"

#include "element.hpp"
#include "image.hpp"
#include "crossing.hpp"

static ElementType current_element = ELEMENT_NORMAL;

/*
函数名称：static int abs_int_local(int value)
功能说明：求整数绝对值
参数说明：
    value：需要求绝对值的输入值
函数返回：输入值的绝对值
修改时间：2026年5月20日
备    注：
example：  abs_int_local(value);
 */
static int abs_int_local(int value)
{
    if(value >= 0)
    {
        return value;
    }

    return -value;
}

/*
函数名称：static int detect_straight(void)
功能说明：检测当前道路是否为长直道
参数说明：无
函数返回：
    1：当前道路可认为是长直道
    0：当前道路不是长直道
修改时间：2026年5月20日
备    注：
    长直道判断主要依据中线波动小、左右丢线少、可视距离足够。
example：  detect_straight();
 */
static int detect_straight(void)
{
    int start_y = image_h - 2;
    int end_y = image_h / 2;

    int sum_error = 0;
    int count = 0;

    if(Search_Stop_Line < image_h / 3)
    {
        return 0;
    }

    if(Both_Lost_Time > 3)
    {
        return 0;
    }

    if(Left_Lost_Time > 8 || Right_Lost_Time > 8)
    {
        return 0;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        if(center_line[y] <= border_min || center_line[y] >= border_max)
        {
            continue;
        }

        sum_error += abs_int_local(center_line[y] - image_w / 2);
        count++;
    }

    if(count <= 0)
    {
        return 0;
    }

    int avg_error = sum_error / count;

    if(avg_error <= 5)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static int detect_cross(void)
功能说明：检测当前道路是否可能为十字元素
参数说明：无
函数返回：
    1：当前道路可能为十字
    0：当前道路不是十字
修改时间：2026年5月20日
备    注：
    当前函数只做十字入口判断，真正的角点查找和补线由 Cross_Detect() 完成。
example：  detect_cross();
 */
static int detect_cross(void)
{
    if(Both_Lost_Time >= 8)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static int detect_obstacle(void)
功能说明：检测当前是否存在障碍物
参数说明：无
函数返回：
    1：检测到障碍物
    0：未检测到障碍物
修改时间：2026年5月20日
备    注：
    当前函数暂时保留接口，后续可以接入 target.cpp 或 obstacle.cpp。
example：  detect_obstacle();
 */
static int detect_obstacle(void)
{
    return 0;
}

/*
函数名称：void element_process(void)
功能说明：元素识别与元素处理总入口，根据当前边线状态判断道路元素，并调用对应处理函数
参数说明：无
函数返回：无
修改时间：2026年5月20日
备    注：
    该函数应在 get_left()、get_right() 之后调用，在 get_center_line() 之前调用。
    因为十字、环岛等元素处理通常会修改 l_border[] 和 r_border[]，必须先补线再求中线。
example：  element_process();
 */
void element_process(void)
{
    current_element = ELEMENT_NORMAL;

    if(detect_obstacle())
    {
        current_element = ELEMENT_OBSTACLE;
        return;
    }

    if(detect_cross())
    {
        Cross_Detect();

        if(Cross_Flaga)
        {
            current_element = ELEMENT_CROSS;
            return;
        }
    }

    if(detect_straight())
    {
        current_element = ELEMENT_STRAIGHT;
        return;
    }

    current_element = ELEMENT_NORMAL;
}

/*
函数名称：ElementType element_get_type(void)
功能说明：获取当前识别到的道路元素类型
参数说明：无
函数返回：当前元素类型
修改时间：2026年5月20日
备    注：
example：  element_get_type();
 */
ElementType element_get_type(void)
{
    return current_element;
}

/*
函数名称：int element_is_straight(void)
功能说明：判断当前是否为长直道
参数说明：无
函数返回：
    1：当前为长直道
    0：当前不是长直道
修改时间：2026年5月20日
备    注：
example：  element_is_straight();
 */
int element_is_straight(void)
{
    return current_element == ELEMENT_STRAIGHT;
}

/*
函数名称：int element_is_cross(void)
功能说明：判断当前是否为十字元素
参数说明：无
函数返回：
    1：当前为十字
    0：当前不是十字
修改时间：2026年5月20日
备    注：
example：  element_is_cross();
 */
int element_is_cross(void)
{
    return current_element == ELEMENT_CROSS;
}

/*
函数名称：int element_is_obstacle(void)
功能说明：判断当前是否为障碍物元素
参数说明：无
函数返回：
    1：当前为障碍物
    0：当前不是障碍物
修改时间：2026年5月20日
备    注：
example：  element_is_obstacle();
 */
int element_is_obstacle(void)
{
    return current_element == ELEMENT_OBSTACLE;
}