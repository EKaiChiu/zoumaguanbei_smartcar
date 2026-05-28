/*********************************************************************************************************************
* 文件名称: main.cpp
* 功能说明: 智能车主程序
* 主要流程:
*   1. 初始化图传、屏幕、摄像头、电机和 PID
*   2. 循环获取摄像头图像
*   3. 二值化后提取边线和中线
*   4. KEY4 发车后执行巡线控制
*   5. 未发车显示菜单，发车后显示运行数据
********************************************************************************************************************/

#include "beep.hpp"
#include "camera.hpp"
#include "circle.hpp"
#include "crossing.hpp"
#include "Display.hpp"
#include "image.hpp"
#include "Key.hpp"
#include "pid.hpp"
#include "target.hpp"
#include "trackline.hpp"
#include "zebra.hpp"
#include "config.hpp"
#include "menu.hpp"
#include "element.hpp"

#include "zf_common_headfile.hpp"
#include "interaction.hpp"
#include <signal.h>


// 0：关闭图传
// 1：TCP 图传
// 2：UDP 图传
#define IMAGE_TRANS_MODE        1

#define SERVER_IP               "192.168.1.104"
#define PORT                    8086

// 1：发送图像 + 左线 + 中线 + 右线
#define INCLUDE_BOUNDARY_TYPE   1


#if(IMAGE_TRANS_MODE == 1)
zf_driver_tcp_client tcp_client_dev;
#endif

#if(IMAGE_TRANS_MODE == 2)
zf_driver_udp udp_dev;
#endif


uint8 *gray_image = NULL;

uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

uint8 x1_boundary[UVC_HEIGHT];
uint8 x2_boundary[UVC_HEIGHT];
uint8 x3_boundary[UVC_HEIGHT];

static volatile sig_atomic_t exit_requested = 0;


#if(IMAGE_TRANS_MODE == 1)
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

uint32 tcp_read_wrap(uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}
#endif


#if(IMAGE_TRANS_MODE == 2)
uint32 udp_send_wrap(const uint8 *buf, uint32 len)
{
    return udp_dev.send_data(buf, len);
}

uint32 udp_read_wrap(uint8 *buf, uint32 len)
{
    return udp_dev.read_data(buf, len);
}
#endif


void sigint_handler(int signum)
{
    (void)signum;
    exit_requested = 1;
}


void stop_all(void)
{
    pit_timer.stop();

    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);

    printf("program exit, motor stop.\r\n");
}


int main(int, char **)
{
    config_init();
    menu_init();

    signal(SIGINT, sigint_handler);


#if(IMAGE_TRANS_MODE == 1)

    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

#elif(IMAGE_TRANS_MODE == 2)

    if(udp_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("udp ok\r\n");
    }
    else
    {
        printf("udp error\r\n");
        return -1;
    }

    seekfree_assistant_interface_init(udp_send_wrap, udp_read_wrap);

#else

    printf("image transfer off\r\n");

#endif


#if(IMAGE_TRANS_MODE != 0)

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        UVC_WIDTH,
        UVC_HEIGHT
    );

#endif


#if((IMAGE_TRANS_MODE != 0) && (INCLUDE_BOUNDARY_TYPE == 1))

    for(int i = 0; i < UVC_HEIGHT; i++)
    {
        x1_boundary[i] = 0;
        x2_boundary[i] = UVC_WIDTH / 2;
        x3_boundary[i] = UVC_WIDTH - 1;
    }

    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        UVC_HEIGHT,
        x1_boundary,
        x2_boundary,
        x3_boundary,
        NULL,
        NULL,
        NULL
    );

#endif


    ips200.init(FB_PATH);

    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    pid_speed_init(
        car_config.speed_kp,
        car_config.speed_ki,
        car_config.speed_kd,
        0,
        0
    );

    trackline_init(
        car_config.track_kp,
        car_config.track_ki,
        car_config.track_kd
    );


    if(uvc_dev.init(UVC_PATH) < 0)
    {
        printf("uvc init error\r\n");
        stop_all();
        return -1;
    }

    printf("uvc init ok\r\n");

    pid_speed_start(20);

    int last_display_page = -1;

    while(!exit_requested)
    {
        if(uvc_dev.wait_image_refresh() < 0)
        {
            printf("camera refresh error\r\n");
            break;
        }

        gray_image = uvc_dev.get_gray_image_ptr();

        if(gray_image == NULL)
        {
            continue;
        }

        // 二值化，gray_image 会被原地改成 0 / 255
        otsu_threshold(gray_image);

        // 提取边线和中线
        image_process_from_bin_ptr(gray_image);

        // 按键和菜单状态
        menu_process();

        // 屏幕刷新降频
        static int display_count = 0;
        display_count++;

        if(display_count >= 5)
        {
            display_count = 0;

            int current_display_page = menu_car_is_started() ? 1 : 0;

            if(current_display_page != last_display_page)
            {
                last_display_page = current_display_page;
                ips200.clear();
            }

            if(menu_car_is_started())
            {
                display_gray_with_pid(gray_image);

                /*
                    如果你已经在 menu.cpp / menu.hpp 里加入了
                    menu_show_run_state()，可以打开这一行显示
                    直线、十字、环岛状态。
                */
                // menu_show_run_state();
            }
            else
            {
                menu_show();
            }
        }


#if(IMAGE_TRANS_MODE != 0)

        // 发二值图到逐飞助手
        memcpy(image_copy[0], gray_image, UVC_WIDTH * UVC_HEIGHT);

    #if(INCLUDE_BOUNDARY_TYPE == 1)

        for(int i = 0; i < UVC_HEIGHT; i++)
        {
            x1_boundary[i] = l_border[i];
            x2_boundary[i] = center_line[i];
            x3_boundary[i] = r_border[i];
        }

    #endif

        seekfree_assistant_camera_send();

#endif


        // KEY4 发车后才下发速度
        if(menu_car_is_started())
        {
            trackline_refresh_wheel_targets(car_config.run_speed, car_config.aim_y);

            pid_speed_set_target(
                trackline_wheel_target_right(),
                trackline_wheel_target_left()
            );
        }
        else
        {
            pid_speed_set_target(0, 0);
        }
    }

    stop_all();

    return 0;
}