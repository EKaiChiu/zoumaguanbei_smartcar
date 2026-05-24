#include "zebra.hpp"
#include "image.hpp"


static int Zebra_Stripes_Flag = 0;
static int zebra_change_count = 0;
static int zebra_narrow_count = 0;


// 限幅函数
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



static int get_standard_road_width(int y)
{
    y = limit_int(y, 0, image_h - 1);

    /*
        简单线性估算：
        顶部约 image_w * 0.25
        底部约 image_w * 0.85

        如果你图像宽 160：
        顶部约 40
        底部约 136
    */
    int min_width = image_w / 4;
    int max_width = image_w * 85 / 100;

    int width = min_width + (max_width - min_width) * y / (image_h - 1);

    return width;
}


/*
    判断某一行左右边界是否可信
*/
static int border_is_valid(int y, uint8 *l_border, uint8 *r_border)
{
    if(y < 0 || y >= image_h)
    {
        return 0;
    }

    int left = l_border[y];
    int right = r_border[y];
    int width = right - left;

    if(left <= border_min + 2)
    {
        return 0;
    }

    if(right >= border_max - 2)
    {
        return 0;
    }

    if(width < image_w / 5)
    {
        return 0;
    }

    if(width > image_w - 5)
    {
        return 0;
    }

    return 1;
}


/*
    斑马线检测

    适配当前工程：
    bin_img   = bin_image
    l_border  = 左边界数组
    r_border  = 右边界数组

    检测逻辑：
    1. 在中下部区域找赛道宽度明显变窄的多行；
    2. 在变窄区域附近统计黑白跳变次数；
    3. 跳变次数足够多，认为是斑马线。
*/
void zebra_stripes_detect(uint8 (*bin_img)[image_w], uint8 *l_border, uint8 *r_border)
{
    Zebra_Stripes_Flag = 0;
    zebra_change_count = 0;
    zebra_narrow_count = 0;

    if(bin_img == NULL || l_border == NULL || r_border == NULL)
    {
        return;
    }

    /*
        只检测中下部区域。
        image_h = 120 时，大约检测 y = 35 ~ 95。
    */
    int y_start = image_h * 4 / 5;
    int y_end   = image_h / 3;

    y_start = limit_int(y_start, 0, image_h - 1);
    y_end   = limit_int(y_end,   0, image_h - 1);

    int valid_line_count = 0;
    int stripe_line_count = 0;
    int black_block_line_count = 0;

    for(int y = y_start; y >= y_end; y--)
    {
        if(!border_is_valid(y, l_border, r_border))
        {
            continue;
        }

        int left = l_border[y] + 3;
        int right = r_border[y] - 3;

        left = limit_int(left, 0, image_w - 2);
        right = limit_int(right, 1, image_w - 1);

        if(right <= left + 10)
        {
            continue;
        }

        valid_line_count++;

        int change_count_line = 0;
        int black_count_line = 0;

        for(int x = left; x < right - 1; x++)
        {
            if(bin_img[y][x] == black_pixel)
            {
                black_count_line++;
            }

            if(bin_img[y][x + 1] != bin_img[y][x])
            {
                change_count_line++;
            }
        }

        zebra_change_count += change_count_line;

        /*
            斑马线特点：
            1. 一行内黑白跳变多；
            2. 赛道内部黑色像素不少；
            3. 多行连续出现。
        */
        int road_width = right - left;

        if(change_count_line >= 4)
        {
            stripe_line_count++;
        }

        if(black_count_line >= road_width / 8)
        {
            black_block_line_count++;
        }
    }

    /*
        这里不要只看总 change_count。
        因为远处噪声也会让 change_count 增大。
        要求多行满足条纹特征更稳。
    */
    if(stripe_line_count >= 3 && black_block_line_count >= 3)
    {
        Zebra_Stripes_Flag = 1;
    }
}

int zebra_stripes_is_found(void)
{
    return Zebra_Stripes_Flag;
}


int zebra_get_change_count(void)
{
    return zebra_change_count;
}


int zebra_get_narrow_count(void)
{
    return zebra_narrow_count;
}