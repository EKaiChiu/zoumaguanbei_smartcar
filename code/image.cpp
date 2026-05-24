#include "image.hpp"
#include "element.hpp"

/*
函数名称：全局图像数据
功能说明：保存二值图、左右边线、中线、赛道宽度以及丢线统计信息
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
example：
 */
uint8 bin_image[image_h][image_w];

uint8 l_border[image_h];
uint8 r_border[image_h];
uint8 center_line[image_h];

int Road_Wide[image_h];

int Search_Stop_Line = 0;

int White_Column[image_w] = {0};

int Longest_White_Column_Left[2] = {0};
int Longest_White_Column_Right[2] = {0};

int Left_Lost_Time = 0;
int Right_Lost_Time = 0;
int Both_Lost_Time = 0;

int Boundry_Start_Left = 0;
int Boundry_Start_Right = 0;

int Left_Lost_Flag[image_h] = {0};
int Right_Lost_Flag[image_h] = {0};

#define LINE_VALID       0
#define LINE_LOST_WHITE  1
#define LINE_BIG_JUMP    2
#define LINE_INVALID_TOP 3

static uint8 Left_Line_State[image_h] = {0};
static uint8 Right_Line_State[image_h] = {0};

static int Valid_Top_Y = 0;

/*
函数名称：static int limit_int(int value, int min_value, int max_value)
功能说明：限制整数变量的取值范围，防止数值超过指定上下限
参数说明：
    value：需要进行限幅处理的输入值
    min_value：允许输出的最小值
    max_value：允许输出的最大值
函数返回：限幅后的整数值
修改时间：2026年5月20日
备    注：
example：  limit_int(value, 0, 100);
 */
static int limit_int(int value, int min_value, int max_value)
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
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
    value：需要求绝对值的整数
函数返回：绝对值
修改时间：2026年5月20日
备    注：
example：  my_abs(x);
 */
int my_abs(int value)
{
    if(value >= 0)
    {
        return value;
    }

    return -value;
}

/*
函数名称：static void copy_bin_image(uint8 *bin_src)
功能说明：将一维二值图拷贝到二维 bin_image 数组
参数说明：
    bin_src：一维二值图指针，图像格式为 0 / 255
函数返回：无
修改时间：2026年5月23日
备    注：
example：  copy_bin_image(gray_image);
 */
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

/*
函数名称：static void image_draw_black_rect(void)
功能说明：给二值图四周加黑框，防止边线搜索时访问越界
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
example：  image_draw_black_rect();
 */
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

/*
函数名称：static void clear_line_data(void)
功能说明：初始化所有边线、中线、宽度和丢线统计数据
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
example：  clear_line_data();
 */
static void clear_line_data(void)
{
    Search_Stop_Line = 0;
    Valid_Top_Y = image_h / 2;

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

        Left_Line_State[y] = LINE_LOST_WHITE;
        Right_Line_State[y] = LINE_LOST_WHITE;
    }

    for(int x = 0; x < image_w; x++)
    {
        White_Column[x] = 0;
    }
}

/*
函数名称：static int get_standard_road_width(int y)
功能说明：根据图像行号估算当前行的赛道宽度，供单边丢线时推算中线使用
参数说明：
    y：图像行号，数值越大表示越靠近图像底部
函数返回：当前行估算得到的赛道宽度，单位为像素
修改时间：2026年5月23日
备    注：
    bottom_width 根据长直道底部实测得到。
    当前实测底部左边线 x=7，右边线 x=146，因此底部赛道宽度为 139。
example：  get_standard_road_width(y);
 */
static int get_standard_road_width(int y)
{
    y = limit_int(y, 0, image_h - 1);

    int top_width = 55;
    int bottom_width = 139;

    int width = top_width + (bottom_width - top_width) * y / (image_h - 1);

    return width;
}

/*
函数名称：void image_filter(uint8(*bin_image)[image_w])
功能说明：对二值图进行简单形态学滤波，填补孤立黑点并去除孤立白点
参数说明：
    bin_image：二维二值图数组，像素值为 0 / 255
函数返回：无
修改时间：2026年5月23日
备    注：
example：  image_filter(bin_image);
 */
#define threshold_max (255 * 5)
#define threshold_min (255 * 2)

void image_filter(uint8(*bin_image)[image_w])
{
    uint16 i;
    uint16 j;
    uint32 num = 0;

    for(i = 1; i < image_h - 1; i++)
    {
        for(j = 1; j < image_w - 1; j++)
        {
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
              + bin_image[i][j - 1]                             + bin_image[i][j + 1]
              + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            if(num >= threshold_max && bin_image[i][j] == black_pixel)
            {
                bin_image[i][j] = white_pixel;
            }

            if(num <= threshold_min && bin_image[i][j] == white_pixel)
            {
                bin_image[i][j] = black_pixel;
            }
        }
    }
}

/*
函数名称：static uint8 is_left_edge(int y, int x)
功能说明：判断指定坐标是否为左边界点
参数说明：
    y：图像行号
    x：图像列号
函数返回：
    1：是左边界
    0：不是左边界
修改时间：2026年5月23日
备    注：
    左边界特征：当前点为白，左侧点为黑。
example：  is_left_edge(y, x);
 */
static uint8 is_left_edge(int y, int x)
{
    if(y <= 0 || y >= image_h)
    {
        return 0;
    }

    if(x <= border_min || x >= border_max)
    {
        return 0;
    }

    if(bin_image[y][x] == white_pixel && bin_image[y][x - 1] == black_pixel)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static uint8 is_right_edge(int y, int x)
功能说明：判断指定坐标是否为右边界点
参数说明：
    y：图像行号
    x：图像列号
函数返回：
    1：是右边界
    0：不是右边界
修改时间：2026年5月23日
备    注：
    右边界特征：当前点为白，右侧点为黑。
example：  is_right_edge(y, x);
 */
static uint8 is_right_edge(int y, int x)
{
    if(y <= 0 || y >= image_h)
    {
        return 0;
    }

    if(x <= border_min || x >= border_max)
    {
        return 0;
    }

    if(bin_image[y][x] == white_pixel && bin_image[y][x + 1] == black_pixel)
    {
        return 1;
    }

    return 0;
}

/*
函数名称：static uint8 find_longest_white_segment(int y, int *left, int *right)
功能说明：在指定行寻找最长连续白色区域，用于车身偏斜时稳定寻找底部赛道区域
参数说明：
    y：需要搜索的图像行号
    left：输出最长白色区域左端点
    right：输出最长白色区域右端点
函数返回：
    1：找到可靠白色区域
    0：未找到可靠白色区域
修改时间：2026年5月23日
备    注：
    该函数用于替代单纯从图像中心左右找边界，提升车身偏斜时的恢复能力。
example：  find_longest_white_segment(y, &left, &right);
 */
static uint8 find_longest_white_segment(int y, int *left, int *right)
{
    int best_start = 0;
    int best_end = 0;
    int best_len = 0;

    int current_start = -1;
    int current_len = 0;

    for(int x = border_min + 2; x <= border_max - 2; x++)
    {
        if(bin_image[y][x] == white_pixel)
        {
            if(current_start < 0)
            {
                current_start = x;
                current_len = 1;
            }
            else
            {
                current_len++;
            }
        }
        else
        {
            if(current_start >= 0)
            {
                if(current_len > best_len)
                {
                    best_len = current_len;
                    best_start = current_start;
                    best_end = x - 1;
                }

                current_start = -1;
                current_len = 0;
            }
        }
    }

    if(current_start >= 0)
    {
        if(current_len > best_len)
        {
            best_len = current_len;
            best_start = current_start;
            best_end = border_max - 2;
        }
    }

    if(best_len < image_w / 5)
    {
        return 0;
    }

    *left = best_start;
    *right = best_end;

    return 1;
}

/*
函数名称：static uint8 find_bottom_border(int *bottom_y)
功能说明：从图像底部多行搜索初始左右边界，为逐行边线追逐建立起点
参数说明：
    bottom_y：输出找到左右边界的行号
函数返回：
    1：找到底部左右边界
    0：未找到可靠底部左右边界
修改时间：2026年5月23日
备    注：
    先找底部最大连续白色区域，即使车身偏斜也能恢复起点。
example：  find_bottom_border(&bottom_y);
 */
static uint8 find_bottom_border(int *bottom_y)
{
    for(int y = image_h - 2; y >= image_h - 18; y--)
    {
        int left = border_min;
        int right = border_max;

        if(find_longest_white_segment(y, &left, &right))
        {
            if(right > left)
            {
                l_border[y] = left;
                r_border[y] = right;

                Road_Wide[y] = right - left;

                Left_Lost_Flag[y] = 0;
                Right_Lost_Flag[y] = 0;

                Left_Line_State[y] = LINE_VALID;
                Right_Line_State[y] = LINE_VALID;

                *bottom_y = y;
                return 1;
            }
        }
    }

    return 0;
}

/*
函数名称：static uint8 search_left_edge_near(int y, int last_x, int scan_range, int *out_x)
功能说明：以上一行左边界为基准，在附近搜索当前行左边界
参数说明：
    y：当前搜索行
    last_x：上一行左边界
    scan_range：左右搜索范围
    out_x：输出找到的左边界横坐标
函数返回：
    1：找到左边界
    0：未找到左边界
修改时间：2026年5月23日
备    注：
    使用上一行边界附近搜索，减少远端噪声干扰。
example：  search_left_edge_near(y, last_left, 10, &left);
 */
static uint8 search_left_edge_near(int y, int last_x, int scan_range, int *out_x)
{
    int low = limit_int(last_x - scan_range, border_min + 1, border_max - 1);
    int high = limit_int(last_x + scan_range, border_min + 1, border_max - 1);

    for(int offset = 0; offset <= scan_range; offset++)
    {
        int x1 = last_x - offset;
        int x2 = last_x + offset;

        if(x1 >= low && x1 <= high)
        {
            if(is_left_edge(y, x1))
            {
                *out_x = x1;
                return 1;
            }
        }

        if(x2 >= low && x2 <= high)
        {
            if(is_left_edge(y, x2))
            {
                *out_x = x2;
                return 1;
            }
        }
    }

    return 0;
}

/*
函数名称：static uint8 search_right_edge_near(int y, int last_x, int scan_range, int *out_x)
功能说明：以上一行右边界为基准，在附近搜索当前行右边界
参数说明：
    y：当前搜索行
    last_x：上一行右边界
    scan_range：左右搜索范围
    out_x：输出找到的右边界横坐标
函数返回：
    1：找到右边界
    0：未找到右边界
修改时间：2026年5月23日
备    注：
    使用上一行边界附近搜索，减少远端噪声干扰。
example：  search_right_edge_near(y, last_right, 10, &right);
 */
static uint8 search_right_edge_near(int y, int last_x, int scan_range, int *out_x)
{
    int low = limit_int(last_x - scan_range, border_min + 1, border_max - 1);
    int high = limit_int(last_x + scan_range, border_min + 1, border_max - 1);

    for(int offset = 0; offset <= scan_range; offset++)
    {
        int x1 = last_x - offset;
        int x2 = last_x + offset;

        if(x1 >= low && x1 <= high)
        {
            if(is_right_edge(y, x1))
            {
                *out_x = x1;
                return 1;
            }
        }

        if(x2 >= low && x2 <= high)
        {
            if(is_right_edge(y, x2))
            {
                *out_x = x2;
                return 1;
            }
        }
    }

    return 0;
}

/*
函数名称：static void fill_invalid_top_area(int top_y)
功能说明：标记顶部无效区域，防止顶部无效边线被画成竖线
参数说明：
    top_y：有效边线顶部行号
函数返回：无
修改时间：2026年5月23日
备    注：
    top_y 以上区域不再复制有效边线，避免显示成竖直假边线。
example：  fill_invalid_top_area(top_y);
 */
static void fill_invalid_top_area(int top_y)
{
    top_y = limit_int(top_y, 0, image_h - 1);

    for(int y = top_y - 1; y >= 0; y--)
    {
        l_border[y] = border_min;
        r_border[y] = border_max;
        center_line[y] = image_w / 2;
        Road_Wide[y] = border_max - border_min;

        Left_Lost_Flag[y] = 1;
        Right_Lost_Flag[y] = 1;

        Left_Line_State[y] = LINE_INVALID_TOP;
        Right_Line_State[y] = LINE_INVALID_TOP;
    }
}
/*
函数名称：static uint8 trace_border_by_previous_line(void)
功能说明：使用逐行边线追逐法提取左右边界，并识别顶部有效行
参数说明：无
函数返回：
    1：边线追逐成功
    0：未找到可靠底部起点
修改时间：2026年5月23日
备    注：
    先找底部最大连续白色区域，再根据上一行边界附近搜索当前行边界。
    当连续丢线或道路宽度异常时，停止向上追边，并记录顶部有效行。
example：  trace_border_by_previous_line();
 */
static uint8 trace_border_by_previous_line(void)
{
    int bottom_y = image_h - 2;

    if(find_bottom_border(&bottom_y) == 0)
    {
        Search_Stop_Line = 0;
        Valid_Top_Y = image_h / 2;
        return 0;
    }

    for(int y = bottom_y + 1; y < image_h; y++)
    {
        l_border[y] = l_border[bottom_y];
        r_border[y] = r_border[bottom_y];

        Road_Wide[y] = r_border[y] - l_border[y];

        Left_Lost_Flag[y] = 0;
        Right_Lost_Flag[y] = 0;

        Left_Line_State[y] = LINE_VALID;
        Right_Line_State[y] = LINE_VALID;
    }

    int last_left = l_border[bottom_y];
    int last_right = r_border[bottom_y];

    int top_limit_y = image_h / 4;
    int valid_top_y = top_limit_y;

    int continuous_both_lost = 0;
    int continuous_bad_width = 0;

    for(int y = bottom_y - 1; y >= top_limit_y; y--)
    {
        int scan_range = 18;

        if(y < image_h * 2 / 3)
        {
            scan_range = 14;
        }

        int left = last_left;
        int right = last_right;

        uint8 left_found = search_left_edge_near(y, last_left, scan_range, &left);
        uint8 right_found = search_right_edge_near(y, last_right, scan_range, &right);

        if(left_found)
        {
            l_border[y] = left;
            Left_Lost_Flag[y] = 0;
            Left_Line_State[y] = LINE_VALID;
            last_left = left;
        }
        else
        {
            l_border[y] = last_left;
            Left_Lost_Flag[y] = 1;
            Left_Line_State[y] = LINE_LOST_WHITE;
        }

        if(right_found)
        {
            r_border[y] = right;
            Right_Lost_Flag[y] = 0;
            Right_Line_State[y] = LINE_VALID;
            last_right = right;
        }
        else
        {
            r_border[y] = last_right;
            Right_Lost_Flag[y] = 1;
            Right_Line_State[y] = LINE_LOST_WHITE;
        }

        Road_Wide[y] = r_border[y] - l_border[y];

        if(!left_found && !right_found)
        {
            continuous_both_lost++;
        }
        else
        {
            continuous_both_lost = 0;
        }

        int standard_width = get_standard_road_width(y);

        if(Road_Wide[y] < standard_width * 35 / 100 ||
           Road_Wide[y] > standard_width * 220 / 100 ||
           l_border[y] >= r_border[y])
        {
            continuous_bad_width++;
        }
        else
        {
            continuous_bad_width = 0;
        }

        if(continuous_both_lost >= 4)
        {
            valid_top_y = y + 4;
            break;
        }

        if(continuous_bad_width >= 4)
        {
            valid_top_y = y + 4;
            break;
        }

        valid_top_y = y;
    }

    valid_top_y = limit_int(valid_top_y, top_limit_y, image_h - 2);

    Valid_Top_Y = valid_top_y;
    Search_Stop_Line = image_h - valid_top_y;

    fill_invalid_top_area(valid_top_y);

    return 1;
}

/*
函数名称：void get_center_line(uint8 hightest)
功能说明：根据边线追逐结果生成中线，支持单边丢线时根据标准赛道宽度补中线
参数说明：
    hightest：中线有效起始行，数值越小表示看得越远
函数返回：无
修改时间：2026年5月23日
备    注：
    配合 trace_border_by_previous_line() 使用。
    根据顶部有效行 Valid_Top_Y 限制中线生成范围。
example：  get_center_line(image_h - Search_Stop_Line);
 */
void get_center_line(uint8 hightest)
{
    int last_center = image_w / 2;

    int center_valid_top = Valid_Top_Y;

    hightest = limit_int(hightest, 0, image_h - 2);

    if(center_valid_top < image_h * 2 / 5)
    {
        center_valid_top = image_h * 2 / 5;
    }

    if(hightest < center_valid_top)
    {
        hightest = center_valid_top;
    }

    Left_Lost_Time = 0;
    Right_Lost_Time = 0;
    Both_Lost_Time = 0;

    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;

    for(int y = 0; y < image_h; y++)
    {
        center_line[y] = image_w / 2;
    }

    for(int y = hightest; y < image_h - 1; y++)
    {
        int left = l_border[y];
        int right = r_border[y];

        int left_valid = Left_Lost_Flag[y] ? 0 : 1;
        int right_valid = Right_Lost_Flag[y] ? 0 : 1;

        if(left <= border_min + 3)
        {
            left_valid = 0;
        }

        if(right >= border_max - 3)
        {
            right_valid = 0;
        }

        if(right <= left)
        {
            left_valid = 0;
            right_valid = 0;
        }

        int standard_width = get_standard_road_width(y);
        int half_width = standard_width / 2;
        int road_width = right - left;

        if(left_valid && right_valid)
        {
            if(road_width < standard_width * 45 / 100)
            {
                left_valid = 0;
                right_valid = 0;
            }

            if(road_width > standard_width * 180 / 100)
            {
                left_valid = 0;
                right_valid = 0;
            }
        }

        Left_Lost_Flag[y] = left_valid ? 0 : 1;
        Right_Lost_Flag[y] = right_valid ? 0 : 1;

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

        int center = last_center;

        if(left_valid && right_valid)
        {
            center = (left + right) >> 1;
            Road_Wide[y] = road_width;
        }
        else if(left_valid && !right_valid)
        {
            center = left + half_width;
            Road_Wide[y] = standard_width;
        }
        else if(!left_valid && right_valid)
        {
            center = right - half_width;
            Road_Wide[y] = standard_width;
        }
        else
        {
            center = last_center;
            Road_Wide[y] = standard_width;
        }

        int max_center_jump = 10;

        if(y < image_h * 2 / 3)
        {
            max_center_jump = 7;
        }

        if(center > last_center + max_center_jump)
        {
            center = last_center + max_center_jump;
        }

        if(center < last_center - max_center_jump)
        {
            center = last_center - max_center_jump;
        }

        center_line[y] = limit_int(center, border_min, border_max);
        last_center = center_line[y];
    }

    Search_Stop_Line = image_h - hightest;
}

/*
函数名称：void image_process(void)
功能说明：图像处理主函数，使用逐行边线追逐法提取左右边线和中线
参数说明：无
函数返回：无
修改时间：2026年5月23日
备    注：
    当前版本不再使用八邻域 search_l_r。
    处理流程为：滤波、加黑框、边线追逐、顶部识别、生成中线、元素处理、重新生成中线。
example：  image_process();
 */
void image_process(void)
{
    image_filter(bin_image);

    image_draw_black_rect();

    if(trace_border_by_previous_line())
    {
        int hightest = image_h - Search_Stop_Line;

        get_center_line((uint8)hightest);

        element_process();

        get_center_line((uint8)hightest);
    }
}

/*
函数名称：void image_process_from_bin_ptr(uint8 *bin_src)
功能说明：图像处理对外主入口，输入一维二值图并提取边线和中线
参数说明：
    bin_src：一维二值图指针，图像必须已经完成二值化
函数返回：无
修改时间：2026年5月23日
备    注：
    main.cpp 中应先调用 otsu_threshold(gray_image)，再调用本函数。
example：  image_process_from_bin_ptr(gray_image);
 */
void image_process_from_bin_ptr(uint8 *bin_src)
{
    if(bin_src == NULL)
    {
        return;
    }

    clear_line_data();

    copy_bin_image(bin_src);

    image_process();
}

/*
函数名称：void image_process_by_white_column(uint8 *bin_src)
功能说明：兼容旧接口，内部调用 image_process_from_bin_ptr
参数说明：
    bin_src：一维二值图指针
函数返回：无
修改时间：2026年5月23日
备    注：
example：  image_process_by_white_column(gray_image);
 */
void image_process_by_white_column(uint8 *bin_src)
{
    image_process_from_bin_ptr(bin_src);
}