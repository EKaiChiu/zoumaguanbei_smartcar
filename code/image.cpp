#include "image.hpp"


// =====================================================
// 全局数组定义
// =====================================================
uint8 bin_image[image_h][image_w];

uint8 l_border[image_h];
uint8 r_border[image_h];
uint8 center_line[image_h];

int Road_Wide[image_h];


int Left_Lost_Time = 0;
int Right_Lost_Time = 0;
int Both_Lost_Time = 0;

int Boundry_Start_Left = 0;//左边界首次有效的行号
int Boundry_Start_Right = 0;//右边界首次有效的行号

int Left_Lost_Flag[image_h] = {0};  //每行左边界是否丢失
int Right_Lost_Flag[image_h] = {0}; //每行右边界是否丢失


// =====================================================
// 限幅函数
// =====================================================
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


// =====================================================
// 拷贝一维二值图到二维 bin_image
// =====================================================
static void copy_bin_image(uint8 *bin_src)
{
    if(bin_src == NULL)
    {
        return;
    }

    for(int y = 0; y < image_h; y++)
    {
        for(int x = 0; x < image_w; x++)
        {
            bin_image[y][x] = bin_src[y * image_w + x];
        }
    }
}


// =====================================================
// 给图像加黑框，防止扫描越界
// =====================================================
static void image_draw_black_rect(void)
{
    for(int y = 0; y < image_h; y++)
    {
        bin_image[y][0] = black_pixel;
        bin_image[y][1] = black_pixel;
        bin_image[y][image_w - 1] = black_pixel;
        bin_image[y][image_w - 2] = black_pixel;
    }

    for(int x = 0; x < image_w; x++)
    {
        bin_image[0][x] = black_pixel;
        bin_image[1][x] = black_pixel;
        bin_image[image_h - 1][x] = black_pixel;
    }
}


// =====================================================
// 初始化所有边界数据
// =====================================================
static void clear_line_data(void)
{
    Search_Stop_Line = 0;

    Left_Lost_Time = 0;
    Right_Lost_Time = 0;
    Both_Lost_Time = 0;

    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;

    Longest_White_Column_Left[0] = 0;
    Longest_White_Column_Left[1] = image_w / 2;

    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = image_w / 2;

    for(int y = 0; y < image_h; y++)
    {
        l_border[y] = border_min;
        r_border[y] = border_max;
        center_line[y] = image_w / 2;

        Road_Wide[y] = border_max - border_min;

        Left_Lost_Flag[y] = 1;
        Right_Lost_Flag[y] = 1;
    }

    for(int x = 0; x < image_w; x++)
    {
        White_Column[x] = 0;
    }
}


// =====================================================
// 根据行号估算道路宽度
// 单边丢线时用这个宽度推中线
// =====================================================
static int get_standard_road_width(int y)
{
    y = limit_int_local(y, 0, image_h - 1);

    /*
        远处赛道窄，近处赛道宽。
        这两个值后面可以根据摄像头实际画面调。
    */
    int top_width = image_w / 3;
    int bottom_width = image_w * 3 / 4;

    int width = top_width + (bottom_width - top_width) * y / (image_h - 1);

    return width;
}


// =====================================================
// 单边丢线时修复中线
// =====================================================
static void repair_center_line_by_single_side(void)
{
    int last_center = image_w / 2;

    for(int y = 0; y < image_h; y++)
    {
        int left = l_border[y];
        int right = r_border[y];

        int left_valid = 1;
        int right_valid = 1;

        if(Left_Lost_Flag[y])
        {
            left_valid = 0;
        }

        if(Right_Lost_Flag[y])
        {
            right_valid = 0;
        }

        if(left <= border_min + 1)
        {
            left_valid = 0;
        }

        if(right >= border_max - 1)
        {
            right_valid = 0;
        }

        if(right <= left)
        {
            left_valid = 0;
            right_valid = 0;
        }

        int half_width = get_standard_road_width(y) / 2;

        if(left_valid && right_valid)
        {
            center_line[y] = (left + right) >> 1;
        }
        else if(left_valid && !right_valid)
        {
            center_line[y] = left + half_width;
        }
        else if(!left_valid && right_valid)
        {
            center_line[y] = right - half_width;
        }
        else
        {
            center_line[y] = last_center;
        }

        center_line[y] = limit_int_local(center_line[y], border_min, border_max);
        last_center = center_line[y];
    }
}


// =====================================================
// 最长白列巡线
// 输入：bin_image，必须已经是 0 / 255
// 输出：l_border / r_border / center_line
// =====================================================
static void longest_white_column_process(void)
{
    int start_column = 20;
    int end_column = image_w - 20;

    int left_border = border_min;
    int right_border = border_max;

    start_column = limit_int_local(start_column, 2, image_w - 3);
    end_column = limit_int_local(end_column, 2, image_w - 3);

    /*
        1. 统计每一列从图像底部向上的连续白色长度
    */
    for(int x = start_column; x <= end_column; x++)
    {
        White_Column[x] = 0;

        for(int y = image_h - 2; y >= 1; y--)
        {
            if(bin_image[y][x] == black_pixel)
            {
                break;
            }
            else
            {
                White_Column[x]++;
            }
        }
    }

    /*
        2. 从左到右找最长白列
    */
    Longest_White_Column_Left[0] = 0;
    Longest_White_Column_Left[1] = image_w / 2;

    for(int x = start_column; x <= end_column; x++)
    {
        if(Longest_White_Column_Left[0] < White_Column[x])
        {
            Longest_White_Column_Left[0] = White_Column[x];
            Longest_White_Column_Left[1] = x;
        }
    }

    /*
        3. 从右到左找最长白列
    */
    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = image_w / 2;

    for(int x = end_column; x >= start_column; x--)
    {
        if(Longest_White_Column_Right[0] < White_Column[x])
        {
            Longest_White_Column_Right[0] = White_Column[x];
            Longest_White_Column_Right[1] = x;
        }
    }

    /*
        4. 搜索截止行
        两边最长白列取较大值，避免搜索范围过短。
    */
    Search_Stop_Line = Longest_White_Column_Left[0];

    if(Longest_White_Column_Right[0] > Search_Stop_Line)
    {
        Search_Stop_Line = Longest_White_Column_Right[0];
    }

    /*
        如果最长白列太短，说明图像很不可靠，保持默认中线。
    */
    if(Search_Stop_Line < 8)
    {
        return;
    }

    int stop_y = image_h - Search_Stop_Line;

    if(stop_y < 0)
    {
        stop_y = 0;
    }

    if(stop_y > image_h - 2)
    {
        stop_y = image_h - 2;
    }

    /*
        5. 从底部向上逐行扫描左右边界
    */
    for(int y = image_h - 2; y >= stop_y; y--)
    {
        /*
            右边界：
            从右侧最长白列向右找 白 黑 黑
        */
        right_border = border_max;
        Right_Lost_Flag[y] = 1;

        for(int x = Longest_White_Column_Right[1]; x <= image_w - 3; x++)
        {
            if(bin_image[y][x] == white_pixel &&
               bin_image[y][x + 1] == black_pixel &&
               bin_image[y][x + 2] == black_pixel)
            {
                right_border = x;
                Right_Lost_Flag[y] = 0;
                break;
            }
        }

        /*
            左边界：
            从左侧最长白列向左找 白 黑 黑 的镜像：
            当前点为白，左边连续两个黑
        */
        left_border = border_min;
        Left_Lost_Flag[y] = 1;

        for(int x = Longest_White_Column_Left[1]; x >= 2; x--)
        {
            if(bin_image[y][x] == white_pixel &&
               bin_image[y][x - 1] == black_pixel &&
               bin_image[y][x - 2] == black_pixel)
            {
                left_border = x;
                Left_Lost_Flag[y] = 0;
                break;
            }
        }

        l_border[y] = left_border;
        r_border[y] = right_border;
    }

    /*
        6. 对未搜索到的上方区域，延续最近边界
    */
    for(int y = stop_y - 1; y >= 0; y--)
    {
        l_border[y] = l_border[y + 1];
        r_border[y] = r_border[y + 1];

        Left_Lost_Flag[y] = Left_Lost_Flag[y + 1];
        Right_Lost_Flag[y] = Right_Lost_Flag[y + 1];
    }

    /*
        7. 统计丢线和赛道宽度
    */
    Left_Lost_Time = 0;
    Right_Lost_Time = 0;
    Both_Lost_Time = 0;

    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;

    for(int y = image_h - 1; y >= 0; y--)
    {
        if(Left_Lost_Flag[y])
        {
            Left_Lost_Time++;
        }

        if(Right_Lost_Flag[y])
        {
            Right_Lost_Time++;
        }

        if(Left_Lost_Flag[y] && Right_Lost_Flag[y])
        {
            Both_Lost_Time++;
        }

        if(Boundry_Start_Left == 0 && !Left_Lost_Flag[y])
        {
            Boundry_Start_Left = y;
        }

        if(Boundry_Start_Right == 0 && !Right_Lost_Flag[y])
        {
            Boundry_Start_Right = y;
        }

        Road_Wide[y] = r_border[y] - l_border[y];
    }

    /*
        8. 生成中线
    */
    repair_center_line_by_single_side();
}


// =====================================================
// 对外主入口
// bin_src 必须已经是二值图
// main.cpp 里应先调用 camera.cpp 的 otsu_threshold()
// =====================================================
void image_process_from_bin_ptr(uint8 *bin_src)
{
    if(bin_src == NULL)
    {
        return;
    }

    clear_line_data();

    copy_bin_image(bin_src);

    image_draw_black_rect();

    longest_white_column_process();
}


// =====================================================
// 兼容旧名字
// =====================================================
void image_process_by_white_column(uint8 *bin_src)
{
    image_process_from_bin_ptr(bin_src);
}