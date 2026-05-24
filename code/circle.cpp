#include "circle.hpp"
#include "image.hpp"
#include "crossing.hpp"

#include <iostream>

#define CIRCLE_DEBUG_ENABLE     1
#define CIRCLE_DEBUG_INTERVAL   10

volatile int Island_State = 0;
volatile int Left_Island_Flag = 0;
volatile int Right_Island_Flag = 0;

volatile int Left_Up_Guai[2] = {0, 0};
volatile int Right_Up_Guai[2] = {0, 0};

static int Left_Down_Guai[2] = {0, 0};
static int Right_Down_Guai[2] = {0, 0};

static int monotonicity_change_line[2] = {0, 0};

static float Circle_Angle = 0.0f;

static int abs_int_local(int value)
{
    if(value >= 0)
    {
        return value;
    }

    return -value;
}

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

static int valid_top_y(void)
{
    int top = image_h - Search_Stop_Line;

    top = limit_int_local(top, 0, image_h - 1);

    return top;
}

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

void circle_set_angle(float angle)
{
    Circle_Angle = angle;
}

void circle_clear(void)
{
    circle_reset();
}

int circle_island_is_found(void)
{
    if(Island_State != 0 || Left_Island_Flag || Right_Island_Flag)
    {
        return 1;
    }

    return 0;
}

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
            k_add_boundary_right(k, island_state_5_down[1], island_state_5_down[0], 0);

            if(Boundry_Start_Right < image_h - 20 || Right_Lost_Time > 10)
            {
                Island_State = 6;
            }
        }
        else if(Island_State == 6)
        {
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