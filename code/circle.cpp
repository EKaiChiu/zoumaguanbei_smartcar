/*
文件名称：circle.cpp
功能说明：环岛识别与补线处理
说明：
    这个文件主要做三件事：
    1. 根据左右边界的突变、单调性变化和丢线情况判断是否进入环岛；
    2. 用 Island_State 维护环岛处理状态；
    3. 在不同状态下对缺失边界进行补线，使后续中线计算还能继续工作。

    坐标约定：
    y 越大越靠近车身，y 越小越靠近图像远端。
    l_border[y] 是第 y 行左边界，r_border[y] 是第 y 行右边界。
    Left_Lost_Flag[y] / Right_Lost_Flag[y] 为 1 表示该行边界不可信。
 */

#include "circle.hpp"
#include "image.hpp"
#include "crossing.hpp"

#include <iostream>

// 调试开关。调车时可以打开，正式跑车建议关闭，避免终端输出影响周期。
#define CIRCLE_DEBUG_ENABLE     1
#define CIRCLE_DEBUG_INTERVAL   10

/*
    环岛状态机变量。
    Island_State = 0 表示不在环岛处理中。
    Left_Island_Flag / Right_Island_Flag 表示当前识别到的是左环还是右环。
 */
volatile int Island_State = 0;
volatile int Left_Island_Flag = 0;
volatile int Right_Island_Flag = 0;

// 上拐点，数组 [0] 存 y 坐标，[1] 存 x 坐标。
volatile int Left_Up_Guai[2] = {0, 0};
volatile int Right_Up_Guai[2] = {0, 0};

// 下拐点，只在本文件内部使用。
static int Left_Down_Guai[2] = {0, 0};
static int Right_Down_Guai[2] = {0, 0};

// 单调性变化点，[0] 存 y 坐标，[1] 存 x 坐标。
static int monotonicity_change_line[2] = {0, 0};

// 环岛累计角度。没有陀螺仪时保持 0，状态切换会更多依赖丢线条件。
static float Circle_Angle = 0.0f;

/*
函数名称：static int abs_int_local(int value)
功能说明：求整数绝对值
参数说明：
    value：输入值
函数返回：value 的绝对值
备注：
    本文件内部使用，避免依赖其他模块的 abs 函数。
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
函数名称：static int limit_int_local(int value, int min_value, int max_value)
功能说明：限制整数范围
参数说明：
    value：输入值
    min_value：下限
    max_value：上限
函数返回：限幅后的值
备注：
    主要用于防止补线坐标越界。
 */
static int limit_int_local(int value, int min_value, int max_value)
{
    if(value < min_value)
    {
        value = min_value;
    }

    if(value > max_value)
    {
        value = max_value;
    }

    return value;
}

/*
函数名称：static int valid_top_y(void)
功能说明：根据 Search_Stop_Line 估计当前有效搜索区域的顶部
参数说明：无
函数返回：有效区域的最小 y 坐标
备注：
    后续找拐点、找突变时不会越过这个顶部区域。
 */
static int valid_top_y(void)
{
    int top = image_h - Search_Stop_Line;

    top = limit_int_local(top, 0, image_h - 1);

    return top;
}

/*
函数名称：static void add_line_to_left_border(int x1, int y1, int x2, int y2)
功能说明：用两点连线方式补左边界
参数说明：
    x1, y1：补线起点
    x2, y2：补线终点
函数返回：无
备注：
    写入 l_border[]，并把对应行 Left_Lost_Flag 清零。
 */
static void add_line_to_left_border(int x1, int y1, int x2, int y2)
{
    int start_y = y1;
    int end_y = y2;

    if(start_y < end_y)
    {
        int temp = start_y;
        start_y = end_y;
        end_y = temp;

        temp = x1;
        x1 = x2;
        x2 = temp;
    }

    start_y = limit_int_local(start_y, 0, image_h - 1);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    if(start_y == end_y)
    {
        l_border[start_y] = (uint8)limit_int_local(x1, border_min, border_max);
        Left_Lost_Flag[start_y] = 0;
        return;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int x = x1 + (x2 - x1) * (y - y1) / (y2 - y1);

        x = limit_int_local(x, border_min, border_max);

        l_border[y] = (uint8)x;
        Left_Lost_Flag[y] = 0;
    }
}

/*
函数名称：static void add_line_to_right_border(int x1, int y1, int x2, int y2)
功能说明：用两点连线方式补右边界
参数说明：
    x1, y1：补线起点
    x2, y2：补线终点
函数返回：无
备注：
    写入 r_border[]，并把对应行 Right_Lost_Flag 清零。
 */
static void add_line_to_right_border(int x1, int y1, int x2, int y2)
{
    int start_y = y1;
    int end_y = y2;

    if(start_y < end_y)
    {
        int temp = start_y;
        start_y = end_y;
        end_y = temp;

        temp = x1;
        x1 = x2;
        x2 = temp;
    }

    start_y = limit_int_local(start_y, 0, image_h - 1);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    if(start_y == end_y)
    {
        r_border[start_y] = (uint8)limit_int_local(x1, border_min, border_max);
        Right_Lost_Flag[start_y] = 0;
        return;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int x = x1 + (x2 - x1) * (y - y1) / (y2 - y1);

        x = limit_int_local(x, border_min, border_max);

        r_border[y] = (uint8)x;
        Right_Lost_Flag[y] = 0;
    }
}

/*
函数名称：static void lengthen_left_border(int start_y, int end_y)
功能说明：按已有左边界趋势向下延长左边界
参数说明：
    start_y：参考起点
    end_y：延长终点
函数返回：无
备注：
    常用于环岛出环阶段，近端边界还没有完全恢复时补线。
 */
static void lengthen_left_border(int start_y, int end_y)
{
    start_y = limit_int_local(start_y, 5, image_h - 6);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    int x1 = l_border[start_y];
    int x2 = l_border[start_y - 5];

    int dx = x1 - x2;

    for(int y = start_y; y <= end_y; y++)
    {
        int x = x1 + dx * (y - start_y) / 5;

        x = limit_int_local(x, border_min, border_max);

        l_border[y] = (uint8)x;
        Left_Lost_Flag[y] = 0;
    }
}

/*
函数名称：static void lengthen_right_border(int start_y, int end_y)
功能说明：按已有右边界趋势向下延长右边界
参数说明：
    start_y：参考起点
    end_y：延长终点
函数返回：无
备注：
    常用于环岛出环阶段，近端边界还没有完全恢复时补线。
 */
static void lengthen_right_border(int start_y, int end_y)
{
    start_y = limit_int_local(start_y, 5, image_h - 6);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    int x1 = r_border[start_y];
    int x2 = r_border[start_y - 5];

    int dx = x1 - x2;

    for(int y = start_y; y <= end_y; y++)
    {
        int x = x1 + dx * (y - start_y) / 5;

        x = limit_int_local(x, border_min, border_max);

        r_border[y] = (uint8)x;
        Right_Lost_Flag[y] = 0;
    }
}

/*
函数名称：static void k_add_boundary_left(float k, int start_x, int start_y, int end_y)
功能说明：按斜率 k 补左边界
参数说明：
    k：y 对 x 的斜率
    start_x：起点 x
    start_y：起点 y
    end_y：终点 y
函数返回：无
备注：
    用于状态 5、6 等需要长距离直线补边的阶段。
 */
static void k_add_boundary_left(float k, int start_x, int start_y, int end_y)
{
    if(k > -0.001f && k < 0.001f)
    {
        return;
    }

    start_y = limit_int_local(start_y, 0, image_h - 1);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    if(start_y < end_y)
    {
        int temp = start_y;
        start_y = end_y;
        end_y = temp;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int x = (int)((y - start_y) / k + start_x);

        x = limit_int_local(x, border_min, border_max);

        l_border[y] = (uint8)x;
        Left_Lost_Flag[y] = 0;
    }
}

/*
函数名称：static void k_add_boundary_right(float k, int start_x, int start_y, int end_y)
功能说明：按斜率 k 补右边界
参数说明：
    k：y 对 x 的斜率
    start_x：起点 x
    start_y：起点 y
    end_y：终点 y
函数返回：无
备注：
    用于状态 5、6 等需要长距离直线补边的阶段。
 */
static void k_add_boundary_right(float k, int start_x, int start_y, int end_y)
{
    if(k > -0.001f && k < 0.001f)
    {
        return;
    }

    start_y = limit_int_local(start_y, 0, image_h - 1);
    end_y = limit_int_local(end_y, 0, image_h - 1);

    if(start_y < end_y)
    {
        int temp = start_y;
        start_y = end_y;
        end_y = temp;
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int x = (int)((y - start_y) / k + start_x);

        x = limit_int_local(x, border_min, border_max);

        r_border[y] = (uint8)x;
        Right_Lost_Flag[y] = 0;
    }
}

/*
函数名称：int Continuity_Change_Left(int start, int end)
功能说明：查找左边界连续性突变点
参数说明：
    start：搜索起始行，通常靠近图像底部
    end：搜索结束行，通常靠近图像上部
函数返回：
    非 0：突变点所在 y 行
    0：未找到突变点
备注：
    左环入口常依赖这个突变点判断。
 */
int Continuity_Change_Left(int start, int end)
{
    if(Left_Lost_Time >= image_h * 9 / 10)
    {
        return 1;
    }

    if(Search_Stop_Line <= 5)
    {
        return 1;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 1; y--)
    {
        if(Left_Lost_Flag[y] || Left_Lost_Flag[y - 1])
        {
            continue;
        }

        if(abs_int_local((int)l_border[y] - (int)l_border[y - 1]) >= 5)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Continuity_Change_Right(int start, int end)
功能说明：查找右边界连续性突变点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：突变点所在 y 行
    0：未找到突变点
备注：
    右环入口常依赖这个突变点判断。
 */
int Continuity_Change_Right(int start, int end)
{
    if(Right_Lost_Time >= image_h * 9 / 10)
    {
        return 1;
    }

    if(Search_Stop_Line <= 5)
    {
        return 1;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 1; y--)
    {
        if(Right_Lost_Flag[y] || Right_Lost_Flag[y - 1])
        {
            continue;
        }

        if(abs_int_local((int)r_border[y] - (int)r_border[y - 1]) >= 8)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Monotonicity_Change_Left(int start, int end)
功能说明：查找左边界单调性变化点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：变化点所在 y 行
    0：未找到变化点
备注：
    可以理解为找左边界局部“拐头”的位置。
 */
int Monotonicity_Change_Left(int start, int end)
{
    if(Left_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    if(start <= end)
    {
        return 0;
    }

    int top = valid_top_y();

    if(end < top + 5)
    {
        end = top + 5;
    }

    for(int y = start; y >= end; y--)
    {
        if(Left_Lost_Flag[y])
        {
            continue;
        }

        int center = l_border[y];

        int all_same = 1;

        for(int k = 1; k <= 5; k++)
        {
            if(l_border[y - k] != center || l_border[y + k] != center)
            {
                all_same = 0;
                break;
            }
        }

        if(all_same)
        {
            continue;
        }

        int local_max = 1;

        for(int k = 1; k <= 5; k++)
        {
            if(center < l_border[y - k] || center < l_border[y + k])
            {
                local_max = 0;
                break;
            }
        }

        if(local_max)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Monotonicity_Change_Right(int start, int end)
功能说明：查找右边界单调性变化点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：变化点所在 y 行
    0：未找到变化点
备注：
    可以理解为找右边界局部“拐头”的位置。
 */
int Monotonicity_Change_Right(int start, int end)
{
    if(Right_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    if(start <= end)
    {
        return 0;
    }

    int top = valid_top_y();

    if(end < top + 5)
    {
        end = top + 5;
    }

    for(int y = start; y >= end; y--)
    {
        if(Right_Lost_Flag[y])
        {
            continue;
        }

        int center = r_border[y];

        int all_same = 1;

        for(int k = 1; k <= 5; k++)
        {
            if(r_border[y - k] != center || r_border[y + k] != center)
            {
                all_same = 0;
                break;
            }
        }

        if(all_same)
        {
            continue;
        }

        int local_min = 1;

        for(int k = 1; k <= 5; k++)
        {
            if(center > r_border[y - k] || center > r_border[y + k])
            {
                local_min = 0;
                break;
            }
        }

        if(local_min)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Find_Left_Down_Point(int start, int end)
功能说明：查找左下拐点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：左下拐点 y 行
    0：未找到
备注：
    主要用于左环入口阶段确定补线起点。
 */
int Find_Left_Down_Point(int start, int end)
{
    if(Left_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 4; y--)
    {
        if(Left_Lost_Flag[y])
        {
            continue;
        }

        if(abs_int_local(l_border[y] - l_border[y + 1]) <= 5 &&
           abs_int_local(l_border[y + 1] - l_border[y + 2]) <= 5 &&
           abs_int_local(l_border[y + 2] - l_border[y + 3]) <= 5 &&
           ((int)l_border[y] - (int)l_border[y - 2]) >= 5 &&
           ((int)l_border[y] - (int)l_border[y - 3]) >= 10 &&
           ((int)l_border[y] - (int)l_border[y - 4]) >= 10)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Find_Left_Up_Point(int start, int end)
功能说明：查找左上拐点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：左上拐点 y 行
    0：未找到
备注：
    主要用于左环内部和出环阶段。
 */
int Find_Left_Up_Point(int start, int end)
{
    if(Left_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 4; y--)
    {
        if(Left_Lost_Flag[y])
        {
            continue;
        }

        if(abs_int_local(l_border[y] - l_border[y - 1]) <= 5 &&
           abs_int_local(l_border[y - 1] - l_border[y - 2]) <= 5 &&
           abs_int_local(l_border[y - 2] - l_border[y - 3]) <= 5 &&
           ((int)l_border[y] - (int)l_border[y + 2]) >= 8 &&
           ((int)l_border[y] - (int)l_border[y + 3]) >= 15 &&
           ((int)l_border[y] - (int)l_border[y + 4]) >= 15)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Find_Right_Down_Point(int start, int end)
功能说明：查找右下拐点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：右下拐点 y 行
    0：未找到
备注：
    主要用于右环入口阶段确定补线起点。
 */
int Find_Right_Down_Point(int start, int end)
{
    if(Right_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 4; y--)
    {
        if(Right_Lost_Flag[y])
        {
            continue;
        }

        if(abs_int_local(r_border[y] - r_border[y + 1]) <= 5 &&
           abs_int_local(r_border[y + 1] - r_border[y + 2]) <= 5 &&
           abs_int_local(r_border[y + 2] - r_border[y + 3]) <= 5 &&
           ((int)r_border[y] - (int)r_border[y - 2]) <= -5 &&
           ((int)r_border[y] - (int)r_border[y - 3]) <= -10 &&
           ((int)r_border[y] - (int)r_border[y - 4]) <= -10)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：int Find_Right_Up_Point(int start, int end)
功能说明：查找右上拐点
参数说明：
    start：搜索起始行
    end：搜索结束行
函数返回：
    非 0：右上拐点 y 行
    0：未找到
备注：
    主要用于右环内部和出环阶段。
 */
int Find_Right_Up_Point(int start, int end)
{
    if(Right_Lost_Time >= image_h * 9 / 10)
    {
        return 0;
    }

    if(start < end)
    {
        int temp = start;
        start = end;
        end = temp;
    }

    start = limit_int_local(start, 5, image_h - 6);
    end = limit_int_local(end, 5, image_h - 6);

    int top = valid_top_y();

    if(end < top)
    {
        end = top;
    }

    for(int y = start; y >= end + 4; y--)
    {
        if(Right_Lost_Flag[y])
        {
            continue;
        }

        if(abs_int_local(r_border[y] - r_border[y - 1]) <= 5 &&
           abs_int_local(r_border[y - 1] - r_border[y - 2]) <= 5 &&
           abs_int_local(r_border[y - 2] - r_border[y - 3]) <= 5 &&
           ((int)r_border[y] - (int)r_border[y + 2]) <= -8 &&
           ((int)r_border[y] - (int)r_border[y + 3]) <= -15 &&
           ((int)r_border[y] - (int)r_border[y + 4]) <= -15)
        {
            return y;
        }
    }

    return 0;
}

/*
函数名称：static void circle_reset(void)
功能说明：清空环岛状态机和所有拐点缓存
参数说明：无
函数返回：无
备注：
    环岛结束、识别失败或强制清除时调用。
 */
static void circle_reset(void)
{
    Island_State = 0;
    Left_Island_Flag = 0;
    Right_Island_Flag = 0;

    Left_Up_Guai[0] = 0;
    Left_Up_Guai[1] = 0;
    Right_Up_Guai[0] = 0;
    Right_Up_Guai[1] = 0;

    Left_Down_Guai[0] = 0;
    Left_Down_Guai[1] = 0;
    Right_Down_Guai[0] = 0;
    Right_Down_Guai[1] = 0;

    monotonicity_change_line[0] = 0;
    monotonicity_change_line[1] = 0;

    Circle_Angle = 0.0f;
}

/*
函数名称：void circle_set_angle(float angle)
功能说明：设置环岛累计角度
参数说明：
    angle：当前累计角度
函数返回：无
备注：
    如果后续接入陀螺仪，可以每帧把角度传进来辅助状态切换。
 */
void circle_set_angle(float angle)
{
    Circle_Angle = angle;
}

/*
函数名称：void circle_clear(void)
功能说明：外部清除环岛状态
参数说明：无
函数返回：无
备注：
    给其他模块调用，内部实际执行 circle_reset()。
 */
void circle_clear(void)
{
    circle_reset();
}

/*
函数名称：int circle_island_is_found(void)
功能说明：判断当前是否正在环岛状态中
参数说明：无
函数返回：
    1：正在识别或处理环岛
    0：未处于环岛状态
备注：
    只要状态机非 0，或左右环标志存在，就认为处于环岛。
 */
int circle_island_is_found(void)
{
    if(Island_State != 0 || Left_Island_Flag || Right_Island_Flag)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static void circle_debug_print(...)
功能说明：按固定间隔打印环岛调试信息
参数说明：
    continuity_change_left_flag：左边界连续性突变点
    continuity_change_right_flag：右边界连续性突变点
    monotonicity_change_left_flag：左边界单调性变化点
    monotonicity_change_right_flag：右边界单调性变化点
函数返回：无
备注：
    调车时使用，正式跑车建议关闭 CIRCLE_DEBUG_ENABLE。
 */
static void circle_debug_print(int continuity_change_left_flag,
                               int continuity_change_right_flag,
                               int monotonicity_change_left_flag,
                               int monotonicity_change_right_flag)
{
#if CIRCLE_DEBUG_ENABLE
    static int debug_count = 0;

    debug_count++;

    if(debug_count < CIRCLE_DEBUG_INTERVAL)
    {
        return;
    }

    debug_count = 0;

    std::cout << "CIRCLE DBG"
              << " State=" << Island_State
              << " LFlag=" << Left_Island_Flag
              << " RFlag=" << Right_Island_Flag
              << " LLost=" << Left_Lost_Time
              << " RLost=" << Right_Lost_Time
              << " BLost=" << Both_Lost_Time
              << " Stop=" << Search_Stop_Line
              << " BSL=" << Boundry_Start_Left
              << " BSR=" << Boundry_Start_Right
              << " CL=" << continuity_change_left_flag
              << " CR=" << continuity_change_right_flag
              << " ML=" << monotonicity_change_left_flag
              << " MR=" << monotonicity_change_right_flag
              << " Angle=" << Circle_Angle
              << std::endl;
#else
    (void)continuity_change_left_flag;
    (void)continuity_change_right_flag;
    (void)monotonicity_change_left_flag;
    (void)monotonicity_change_right_flag;
#endif
}

/*
函数名称：static int try_enter_left_circle(...)
功能说明：尝试进入左环岛状态
参数说明：
    continuity_change_left_flag：左边界突变点
    continuity_change_right_flag：右边界突变点
    monotonicity_change_right_flag：右边界单调性变化点
函数返回：
    1：成功进入左环
    0：条件不足，未进入
备注：
    这里是左环入口判断的核心位置。
 */
static int try_enter_left_circle(int continuity_change_left_flag,
                                 int continuity_change_right_flag,
                                 int monotonicity_change_right_flag)
{
    if(monotonicity_change_right_flag == 0 &&
       continuity_change_left_flag != 0 &&
       Left_Lost_Time >= 6 &&
       Left_Lost_Time <= image_h * 7 / 10 &&
       Right_Lost_Time <= 25 &&
       Search_Stop_Line >= image_h * 55 / 100 &&
       Both_Lost_Time <= 25)
    {
        Left_Down_Guai[0] = Find_Left_Down_Point(image_h - 1, image_h / 6);

        if(Left_Down_Guai[0] < image_h / 4)
        {
            Left_Down_Guai[0] = continuity_change_left_flag;
        }

        Left_Down_Guai[0] = limit_int_local(Left_Down_Guai[0], 0, image_h - 1);
        Left_Down_Guai[1] = l_border[Left_Down_Guai[0]];

        if(Left_Down_Guai[0] >= image_h / 4)
        {
            Island_State = 1;
            Left_Island_Flag = 1;
            Right_Island_Flag = 0;

            return 1;
        }

        circle_reset();
    }

    (void)continuity_change_right_flag;

    return 0;
}

/*
函数名称：static int try_enter_right_circle(...)
功能说明：尝试进入右环岛状态
参数说明：
    continuity_change_right_flag：右边界突变点
    continuity_change_left_flag：左边界突变点
    monotonicity_change_left_flag：左边界单调性变化点
函数返回：
    1：成功进入右环
    0：条件不足，未进入
备注：
    这里是右环入口判断的核心位置。
 */
static int try_enter_right_circle(int continuity_change_right_flag,
                                  int continuity_change_left_flag,
                                  int monotonicity_change_left_flag)
{
    if(monotonicity_change_left_flag == 0 &&
       continuity_change_right_flag != 0 &&
       Right_Lost_Time >= 6 &&
       Right_Lost_Time <= image_h * 7 / 10 &&
       Left_Lost_Time <= 25 &&
       Search_Stop_Line >= image_h * 55 / 100 &&
       Both_Lost_Time <= 25)
    {
        Right_Down_Guai[0] = Find_Right_Down_Point(image_h - 1, image_h / 6);

        if(Right_Down_Guai[0] < image_h / 4)
        {
            Right_Down_Guai[0] = continuity_change_right_flag;
        }

        Right_Down_Guai[0] = limit_int_local(Right_Down_Guai[0], 0, image_h - 1);
        Right_Down_Guai[1] = r_border[Right_Down_Guai[0]];

        if(Right_Down_Guai[0] >= image_h / 4)
        {
            Island_State = 1;
            Right_Island_Flag = 1;
            Left_Island_Flag = 0;

            return 1;
        }

        circle_reset();
    }

    (void)continuity_change_left_flag;

    return 0;
}

/*
函数名称：void Island_Detect(void)
功能说明：环岛识别和状态机处理入口
参数说明：无
函数返回：无
备注：
    每帧边界提取完成后调用。
    进入环岛后，根据 Island_State 逐步完成入口补线、环内补线、出环补线和复位。
 */
void Island_Detect(void)
{
    static float k = 0.0f;

    static int island_state_5_down[2] = {0, 0};
    static int island_state_3_up[2] = {0, 0};

    int monotonicity_change_left_flag = 0;
    int monotonicity_change_right_flag = 0;
    int continuity_change_left_flag = 0;
    int continuity_change_right_flag = 0;

    continuity_change_left_flag = Continuity_Change_Left(image_h - 6, image_h / 3);
    continuity_change_right_flag = Continuity_Change_Right(image_h - 6, image_h / 3);

    monotonicity_change_left_flag = Monotonicity_Change_Left(image_h - 10, image_h / 3);
    monotonicity_change_right_flag = Monotonicity_Change_Right(image_h - 10, image_h / 3);

    circle_debug_print(continuity_change_left_flag,
                       continuity_change_right_flag,
                       monotonicity_change_left_flag,
                       monotonicity_change_right_flag);

    if(Cross_Flaga == 0 && Island_State == 0)
    {
        if(Left_Island_Flag == 0)
        {
            if(try_enter_left_circle(continuity_change_left_flag,
                                     continuity_change_right_flag,
                                     monotonicity_change_right_flag))
            {
                std::cout << "CIRCLE ENTER LEFT" << std::endl;
            }
        }

        if(Right_Island_Flag == 0 && Island_State == 0)
        {
            if(try_enter_right_circle(continuity_change_right_flag,
                                      continuity_change_left_flag,
                                      monotonicity_change_left_flag))
            {
                std::cout << "CIRCLE ENTER RIGHT" << std::endl;
            }
        }
    }

    if(Left_Island_Flag == 1)
    {
        /*
            左环状态 1：
            刚进入左环。此时左边界通常开始丢失或向外突变，
            先根据左边界变化点补一条左边线，保证中线不会断。
         */
        if(Island_State == 1)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Left(image_h / 4, 5);

            if(monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = l_border[monotonicity_change_line[0]];

                add_line_to_left_border(
                    (int)(monotonicity_change_line[1] * 15 / 100),
                    image_h - 1,
                    monotonicity_change_line[1],
                    monotonicity_change_line[0]
                );
            }
            else
            {
                add_line_to_left_border(border_min + 5, image_h - 1, l_border[image_h / 2], image_h / 2);
            }

            if(Boundry_Start_Left < image_h * 75 / 100 || Left_Lost_Time > 12)
            {
                Island_State = 2;
            }
        }
        else if(Island_State == 2)
        {
            /*
                左环状态 2：
                继续沿左侧缺失区域补线，等待车辆更深入环岛。
             */
            monotonicity_change_line[0] = Monotonicity_Change_Left(image_h - 20, image_h / 3);

            if(monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = l_border[monotonicity_change_line[0]];

                add_line_to_left_border(
                    (int)(monotonicity_change_line[1] * 10 / 100),
                    image_h - 1,
                    monotonicity_change_line[1],
                    monotonicity_change_line[0]
                );
            }
            else
            {
                add_line_to_left_border(border_min + 5, image_h - 1, l_border[image_h / 2], image_h / 2);
            }

            if(monotonicity_change_line[0] > image_h * 50 / 100 ||
               Left_Lost_Time > 18 ||
               Both_Lost_Time <= 5)
            {
                Island_State = 3;
            }
        }
        else if(Island_State == 3)
        {
            /*
                左环状态 3：
                尝试寻找左上拐点，并从图像右下方向左上拐点补线。
                这一步主要让车头向环内方向走。
             */
            if(k == 0.0f)
            {
                Left_Up_Guai[0] = Find_Left_Up_Point(image_h / 2, 5);
                Left_Up_Guai[0] = limit_int_local(Left_Up_Guai[0], 0, image_h - 1);
                Left_Up_Guai[1] = l_border[Left_Up_Guai[0]];

                if(Left_Up_Guai[0] >= image_h / 8 &&
                   Left_Up_Guai[0] < image_h * 2 / 3 &&
                   Left_Up_Guai[1] > image_w / 4 &&
                   Left_Up_Guai[1] < image_w * 4 / 5)
                {
                    island_state_3_up[0] = Left_Up_Guai[0];
                    island_state_3_up[1] = Left_Up_Guai[1];

                    int dx = image_w - 20 - island_state_3_up[1];

                    if(dx != 0)
                    {
                        k = (float)(image_h - island_state_3_up[0]) / (float)dx;
                    }
                }
            }

            if(k != 0.0f)
            {
                add_line_to_left_border(image_w - 25, image_h - 10, island_state_3_up[1], island_state_3_up[0]);
            }
            else
            {
                add_line_to_left_border(image_w - 25, image_h - 10, l_border[image_h / 2], image_h / 2);
            }

            if(abs_int_local((int)Circle_Angle) >= 50 ||
               Both_Lost_Time <= 3 ||
               Left_Lost_Time <= 8)
            {
                k = 0.0f;
                Island_State = 4;
            }
        }
        else if(Island_State == 4)
        {
            /*
                左环状态 4：
                等待右侧边界出现合适的单调性变化点，为出环侧补线做准备。
             */
            if(abs_int_local((int)Circle_Angle) > 180 ||
               Right_Lost_Time <= 8 ||
               Left_Lost_Time <= 8)
            {
                monotonicity_change_line[0] = Monotonicity_Change_Right(image_h - 10, 10);

                if(monotonicity_change_line[0] >= image_h / 4 &&
                   monotonicity_change_line[0] <= image_h * 2 / 3)
                {
                    monotonicity_change_line[1] = r_border[monotonicity_change_line[0]];

                    island_state_5_down[0] = image_h - 1;
                    island_state_5_down[1] = limit_int_local(r_border[image_h - 1] + 10, border_min, border_max);

                    int dx = island_state_5_down[1] - monotonicity_change_line[1];

                    if(dx != 0)
                    {
                        k = (float)(image_h - monotonicity_change_line[0]) / (float)dx;
                        Island_State = 5;
                    }
                }
            }
        }
        else if(Island_State == 5)
        {
            /*
                左环状态 5：
                使用斜率 k 补右边界，开始准备从环岛内部过渡到出口。
             */
            k_add_boundary_right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if(Boundry_Start_Right < image_h - 20 || Right_Lost_Time > 10)
            {
                Island_State = 6;
            }
        }
        else if(Island_State == 6)
        {
            /*
                左环状态 6：
                继续补右边界，等待出口边线恢复。
             */
            k_add_boundary_right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if(Boundry_Start_Right > image_h - 10 ||
               Right_Lost_Time <= 5 ||
               abs_int_local((int)Circle_Angle) >= 300)
            {
                k = 0.0f;
                Island_State = 7;
            }
        }
        else if(Island_State == 7)
        {
            /*
                左环状态 7：
                查找左上拐点，判断是否已经接近出环。
             */
            Left_Up_Guai[0] = Find_Left_Up_Point(image_h - 10, 5);
            Left_Up_Guai[0] = limit_int_local(Left_Up_Guai[0], 0, image_h - 1);
            Left_Up_Guai[1] = l_border[Left_Up_Guai[0]];

            if(Left_Up_Guai[0] >= 5 &&
               Left_Up_Guai[0] <= image_h - 20 &&
               Left_Up_Guai[1] <= image_w * 4 / 5)
            {
                Island_State = 8;
            }
        }
        else if(Island_State == 8)
        {
            /*
                左环状态 8：
                左边界逐步恢复，向下延长边界并准备退出环岛状态。
             */
            Left_Up_Guai[0] = Find_Left_Up_Point(image_h - 1, 10);

            if(Left_Up_Guai[0] > 5)
            {
                lengthen_left_border(Left_Up_Guai[0] - 1, image_h - 1);
            }

            if(Left_Up_Guai[0] >= image_h - 20 ||
               (Left_Up_Guai[0] < 10 && Boundry_Start_Left >= image_h - 10))
            {
                Island_State = 9;
                Left_Island_Flag = 0;
                k = 0.0f;
                Circle_Angle = 0.0f;
            }
        }
    }
    else if(Right_Island_Flag == 1)
    {
        /*
            右环状态 1：
            刚进入右环。右边界开始丢失或向外突变，
            先补右边线，保证中线可用。
         */
        if(Island_State == 1)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Right(image_h / 4, 5);

            if(monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = r_border[monotonicity_change_line[0]];

                add_line_to_right_border(
                    image_w - 1 - (image_w - monotonicity_change_line[1]) * 15 / 100,
                    image_h - 1,
                    monotonicity_change_line[1],
                    monotonicity_change_line[0]
                );
            }
            else
            {
                add_line_to_right_border(border_max - 5, image_h - 1, r_border[image_h / 2], image_h / 2);
            }

            if(Boundry_Start_Right < image_h * 75 / 100 || Right_Lost_Time > 12)
            {
                Island_State = 2;
            }
        }
        else if(Island_State == 2)
        {
            monotonicity_change_line[0] = Monotonicity_Change_Right(image_h - 20, image_h / 3);

            if(monotonicity_change_line[0] > 0)
            {
                monotonicity_change_line[1] = r_border[monotonicity_change_line[0]];

                add_line_to_right_border(
                    image_w - 1 - (image_w - monotonicity_change_line[1]) * 15 / 100,
                    image_h - 1,
                    monotonicity_change_line[1],
                    monotonicity_change_line[0]
                );
            }
            else
            {
                add_line_to_right_border(border_max - 5, image_h - 1, r_border[image_h / 2], image_h / 2);
            }

            if(monotonicity_change_line[0] > image_h / 3 ||
               Right_Lost_Time > 18 ||
               Both_Lost_Time <= 5)
            {
                Island_State = 3;
            }
        }
        else if(Island_State == 3)
        {
            if(k == 0.0f)
            {
                Right_Up_Guai[0] = Find_Right_Up_Point(image_h / 2, 5);
                Right_Up_Guai[0] = limit_int_local(Right_Up_Guai[0], 0, image_h - 1);
                Right_Up_Guai[1] = r_border[Right_Up_Guai[0]];

                if(Right_Up_Guai[0] >= image_h / 8 &&
                   Right_Up_Guai[0] < image_h * 2 / 3 &&
                   Right_Up_Guai[1] > image_w / 5 &&
                   Right_Up_Guai[1] < image_w - 5)
                {
                    island_state_3_up[0] = Right_Up_Guai[0];
                    island_state_3_up[1] = Right_Up_Guai[1];

                    int dx = 20 - island_state_3_up[1];

                    if(dx != 0)
                    {
                        k = (float)(image_h - island_state_3_up[0]) / (float)dx;
                    }
                }
            }

            if(k != 0.0f)
            {
                add_line_to_right_border(25, image_h - 10, island_state_3_up[1], island_state_3_up[0]);
            }
            else
            {
                add_line_to_right_border(25, image_h - 10, r_border[image_h / 2], image_h / 2);
            }

            if(abs_int_local((int)Circle_Angle) >= 50 ||
               Both_Lost_Time <= 3 ||
               Right_Lost_Time <= 8)
            {
                k = 0.0f;
                Island_State = 4;
            }
        }
        else if(Island_State == 4)
        {
            if(abs_int_local((int)Circle_Angle) > 180 ||
               Left_Lost_Time <= 8 ||
               Right_Lost_Time <= 8)
            {
                monotonicity_change_line[0] = Monotonicity_Change_Left(image_h - 10, 10);

                if(monotonicity_change_line[0] >= image_h / 4 &&
                   monotonicity_change_line[0] <= image_h * 2 / 3)
                {
                    monotonicity_change_line[1] = l_border[monotonicity_change_line[0]];

                    island_state_5_down[0] = image_h - 1;
                    island_state_5_down[1] = limit_int_local(l_border[image_h - 1] - 5, border_min, border_max);

                    int dx = island_state_5_down[1] - monotonicity_change_line[1];

                    if(dx != 0)
                    {
                        k = (float)(image_h - monotonicity_change_line[0]) / (float)dx;
                        Island_State = 5;
                    }
                }
            }
        }
        else if(Island_State == 5)
        {
            k_add_boundary_left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if(Boundry_Start_Left < image_h - 20 || Left_Lost_Time > 10)
            {
                Island_State = 6;
            }
        }
        else if(Island_State == 6)
        {
            k_add_boundary_left(k, island_state_5_down[1], island_state_5_down[0], 0);

            if(Boundry_Start_Left > image_h - 10 ||
               Left_Lost_Time <= 5 ||
               abs_int_local((int)Circle_Angle) >= 300)
            {
                k = 0.0f;
                Island_State = 7;
            }
        }
        else if(Island_State == 7)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(image_h - 10, 5);
            Right_Up_Guai[0] = limit_int_local(Right_Up_Guai[0], 0, image_h - 1);
            Right_Up_Guai[1] = r_border[Right_Up_Guai[0]];

            if(Right_Up_Guai[0] >= 5 &&
               Right_Up_Guai[0] <= image_h - 20 &&
               Right_Up_Guai[1] >= image_w / 5)
            {
                Island_State = 8;
            }
        }
        else if(Island_State == 8)
        {
            Right_Up_Guai[0] = Find_Right_Up_Point(image_h - 1, 10);

            if(Right_Up_Guai[0] > 5)
            {
                lengthen_right_border(Right_Up_Guai[0] - 1, image_h - 1);
            }

            if(Right_Up_Guai[0] >= image_h - 20 ||
               (Right_Up_Guai[0] < 10 && Boundry_Start_Right >= image_h - 10))
            {
                Island_State = 9;
                Right_Island_Flag = 0;
                k = 0.0f;
                Circle_Angle = 0.0f;
            }
        }
    }

    if(abs_int_local((int)Circle_Angle) > 360)
    {
        Island_State = 9;
        Circle_Angle = 0.0f;
    }

    if(Island_State == 9)
    {
        std::cout << "CIRCLE EXIT" << std::endl;
        circle_reset();
 
    }
}