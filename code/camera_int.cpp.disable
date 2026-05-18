#include "camera.hpp"

#define GRAY_SCALE 256

static uint8 image_threshold = 0;//最佳阈值


/*
    大津法阈值计算

    输入：
        image：一维灰度图
        col：图像宽度
        row：图像高度

    输出：
        最佳二值化阈值
*/
static uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
    if(image == NULL)
    {
        return 0;
    }

    uint16 image_width  = col;
    uint16 image_height = row;

    uint32 hist[GRAY_SCALE] = {0};

    uint32 total_pixels = 0;
    uint32 weight_back = 0;
    uint32 weight_fore = 0;

    uint64 sum_total = 0;
    uint64 sum_back = 0;

    uint16 min_value = 0;
    uint16 max_value = 255;

    uint8 threshold = 0;

    /*
        1. 统计灰度直方图
    */
    for(uint16 y = 0; y < image_height; y++)
    {
        for(uint16 x = 0; x < image_width; x++)
        {
            uint8 gray = image[y * image_width + x];
            hist[gray]++;
        }
    }

    /*
        2. 找最小灰度值
    */
    for(min_value = 0; min_value < 256 && hist[min_value] == 0; min_value++)
    {
    }

    /*
        3. 找最大灰度值
    */
    for(max_value = 255; max_value > min_value && hist[max_value] == 0; max_value--)
    {
    }

    /*
        图像只有一种灰度
    */
    if(max_value == min_value)
    {
        return (uint8)max_value;
    }

    /*
        图像只有两个相邻灰度
    */
    if(min_value + 1 == max_value)
    {
        return (uint8)min_value;
    }

    /*
        4. 统计像素总数和灰度总和
    */
    for(uint16 i = min_value; i <= max_value; i++)
    {
        total_pixels += hist[i];
        sum_total += (uint64)hist[i] * i;
    }

    if(total_pixels == 0)
    {
        return 0;
    }

    /*
        5. 遍历阈值，寻找最大类间方差

        使用整数形式：
        score = (total * sum_back - weight_back * sum_total)^2
                / (weight_back * weight_fore)

        为了避免浮点和除法，用交叉相乘比较：
        num / den > best_num / best_den
        即：
        num * best_den > best_num * den
    */
    __int128 best_num = -1;
    __int128 best_den = 1;

    sum_back = 0;
    weight_back = 0;

    for(uint16 i = min_value; i < max_value; i++)
    {
        weight_back += hist[i];
        sum_back += (uint64)hist[i] * i;

        weight_fore = total_pixels - weight_back;

        if(weight_back == 0 || weight_fore == 0)
        {
            continue;
        }

        /*
            diff = total_pixels * sum_back - weight_back * sum_total
        */
        __int128 diff =
            (__int128)total_pixels * sum_back
          - (__int128)weight_back * sum_total;

        __int128 num = diff * diff;
        __int128 den = (__int128)weight_back * weight_fore;

        if(best_num < 0 || num * best_den > best_num * den)
        {
            best_num = num;
            best_den = den;
            threshold = (uint8)i;
        }
    }

    return threshold;
}


/*
    对外二值化函数

    main.cpp 里调用：
        otsu_threshold(gray_image);

    调用后：
        gray_image 会被直接改成 0 / 255
*/
void otsu_threshold(uint8 *gray_image)
{
    if(gray_image == NULL)
    {
        return;
    }

    image_threshold = otsuThreshold(gray_image, UVC_WIDTH, UVC_HEIGHT);

    for(int i = 0; i < UVC_WIDTH * UVC_HEIGHT; i++)
    {
        if(gray_image[i] > image_threshold)
        {
            gray_image[i] = 255;
        }
        else
        {
            gray_image[i] = 0;
        }
    }
}


/*
    获取当前阈值，方便屏幕显示调试
*/
uint8 camera_get_threshold(void)
{
    return image_threshold;
}