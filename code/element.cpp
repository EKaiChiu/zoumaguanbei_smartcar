#include "element.hpp"
#include "image.hpp"
#include "crossing.hpp"
#include "zebra.hpp"
#include "circle.hpp"

static ElementType current_element = ELEMENT_NORMAL;

/*
函数名称：static int abs_int_local(int value)
功能说明：求整数绝对值
参数说明：
    value：需要求绝对值的输入值
函数返回：输入值的绝对值
修改时间：2026年5月24日
备注：
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
修改时间：2026年5月24日
备注：
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

    if(circle_island_is_found())
    {
        return 0;
    }

    if(Cross_Flaga)
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
修改时间：2026年5月24日
备注：
    当前函数只做十字入口判断，真正的角点查找和补线由 Cross_Detect() 完成。
example：  detect_cross();
 */
static int detect_cross(void)
{
    if(circle_island_is_found())
    {
        return 0;
    }

    if(Island_State != 0)
    {
        return 0;
    }

    if(Both_Lost_Time >= 8)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static int detect_circle(void)
功能说明：检测并处理环岛元素
参数说明：无
函数返回：
    1：当前处于环岛状态
    0：当前不处于环岛状态
修改时间：2026年5月24日
备注：
    Island_Detect() 内部会根据边线状态修改 l_border[] / r_border[] 完成补线。
example：  detect_circle();
 */
static int detect_circle(void)
{
    if(Cross_Flaga)
    {
        return 0;
    }

    Island_Detect();

    if(circle_island_is_found())
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static int detect_zebra(void)
功能说明：检测当前是否识别到斑马线
参数说明：无
函数返回：
    1：检测到斑马线
    0：未检测到斑马线
修改时间：2026年5月24日
备注：
    斑马线检测依赖 bin_image、l_border、r_border。
example：  detect_zebra();
 */
static int detect_zebra(void)
{
    zebra_stripes_detect(bin_image, l_border, r_border);

    if(zebra_stripes_is_found())
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
修改时间：2026年5月24日
备注：
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
修改时间：2026年5月24日
备注：
    处理优先级：
    障碍物 > 斑马线 > 十字 > 环岛 > 长直道 > 普通道路。
    十字和环岛会修改边线，因此调用 element_process() 后应重新生成中线。
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

    if(detect_zebra())
    {
        current_element = ELEMENT_ZEBRA;
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

    // if(detect_circle())
    // {
    //     current_element = ELEMENT_CIRCLE;
    //     return;
    // }

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
修改时间：2026年5月24日
备注：
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
修改时间：2026年5月24日
备注：
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
修改时间：2026年5月24日
备注：
example：  element_is_cross();
 */
int element_is_cross(void)
{
    return current_element == ELEMENT_CROSS;
}

/*
函数名称：int element_is_circle(void)
功能说明：判断当前是否为环岛元素
参数说明：无
函数返回：
    1：当前为环岛
    0：当前不是环岛
修改时间：2026年5月24日
备注：
example：  element_is_circle();
 */
int element_is_circle(void)
{
    return current_element == ELEMENT_CIRCLE;
}

/*
函数名称：int element_is_zebra(void)
功能说明：判断当前是否为斑马线元素
参数说明：无
函数返回：
    1：当前为斑马线
    0：当前不是斑马线
修改时间：2026年5月24日
备注：
example：  element_is_zebra();
 */
int element_is_zebra(void)
{
    return current_element == ELEMENT_ZEBRA;
}

/*
函数名称：int element_is_obstacle(void)
功能说明：判断当前是否为障碍物元素
参数说明：无
函数返回：
    1：当前为障碍物
    0：当前不是障碍物
修改时间：2026年5月24日
备注：
example：  element_is_obstacle();
 */
int element_is_obstacle(void)
{
    return current_element == ELEMENT_OBSTACLE;
}