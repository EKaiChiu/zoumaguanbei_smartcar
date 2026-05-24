#include "Display.hpp"
#include "pid.hpp"



// 显示 PID 参数
void display_pid_data(void)
{   ips200.show_string(0, 125, "智能车");
    // ips200.show_string(0, 125, "RT:");
    // ips200.show_int(30, 125, pid_get_target_right(), 4);

    ips200.show_string(80, 125, "LT:");
    ips200.show_int(110, 125, pid_get_target_left(), 4);

    ips200.show_string(0, 145, "RE:");
    ips200.show_int(30, 145, encoder_right, 4);

    ips200.show_string(80, 145, "LE:");
    ips200.show_int(110, 145, encoder_left, 4);

    ips200.show_string(0, 165, "RO:");
    ips200.show_int(30, 165, (int)RSPID.output, 4);

    ips200.show_string(80, 165, "LO:");
    ips200.show_int(110, 165, (int)LSPID.output, 4);
}


// 显示二值图/灰度图 + PID 参数
void display_gray_with_pid(uint8 *image)
{
    // if(image != NULL)
    // {
    //     ips200.displayimage_gray(image, UVC_WIDTH, UVC_HEIGHT);
    // }

    display_pid_data();
}