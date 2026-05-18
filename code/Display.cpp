#include "Display.hpp"
#include "pid.hpp"


/*
void display_image(ImageType type)
{
    if(uvc_dev.wait_image_refresh() == 0)
    {
        if(type == RGB)
        {
            uint16* rgb_image = uvc_dev.get_rgb_image_ptr();

            if(NULL != rgb_image)
            {
                ips200.displayimage_rgb565(rgb_image, UVC_WIDTH, UVC_HEIGHT);
            }
        }
        else if(type == GRAY)
        {
            uint8* gray_image = uvc_dev.get_gray_image_ptr();

            if(NULL != gray_image)
            {
                ips200.displayimage_gray(gray_image, UVC_WIDTH, UVC_HEIGHT);
            }
        }
    }
}
*/

// 显示 PID 参数
void display_pid_data(void)
{
    ips200.show_string(0, 125, "RT:");
    ips200.show_int(30, 125, pid_get_target_right(), 4);

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
    if(image != NULL)
    {
        ips200.displayimage_gray(image, UVC_WIDTH, UVC_HEIGHT);
    }

    display_pid_data();
}