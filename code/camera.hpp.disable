#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "zf_common_headfile.hpp"
#include "../user/interaction.hpp"

// 大津法二值化
// 输入：UVC 灰度图指针
// 输出：gray_image 原地变成 0 / 255 二值图
void otsu_threshold(uint8 *gray_image);

// 获取当前二值化阈值，方便显示调试
uint8 camera_get_threshold(void);

#endif // CAMERA_HPP