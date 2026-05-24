/*********************************************************************************************************************
* 文件名称: main
* 功能说明: 基础版主程序
* 功能流程:
*   1. 根据 IMAGE_TRANS_MODE 选择关闭图传 / TCP 图传 / UDP 图传
*   2. 初始化逐飞助手通信接口
*   3. 配置摄像头图像 + 三条 X 边界
*   4. 初始化屏幕、摄像头、电机、PID
*   5. 循环取图、二值化、八邻域找边界、中线控制、图传发送
********************************************************************************************************************/

#include "beep.hpp"
#include "camera.hpp"
//#include "circle.hpp"
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

#include "zf_common_headfile.hpp"
#include "interaction.hpp"
#include <chrono>
#include <iostream>
#include <signal.h>


// ====================== 图传模式选择 ======================
// 0：关闭图传
// 1：TCP 图传
// 2：UDP 图传
#define IMAGE_TRANS_MODE    1


// ====================== TCP / UDP 配置 ======================
#define SERVER_IP   "192.168.1.104"     // 电脑端逐飞助手所在 IP
#define PORT        8086                // 与逐飞助手端口一致


// ====================== 边界发送类型 ======================
// 先用 TYPE 1：发送图像 + 三条 X 边界
// x1_boundary = 左边线
// x2_boundary = 中线
// x3_boundary = 右边线
#define INCLUDE_BOUNDARY_TYPE   1


// ====================== 全局对象与缓冲区 ======================
#if(IMAGE_TRANS_MODE == 1)
zf_driver_tcp_client tcp_client_dev;
#endif

#if(IMAGE_TRANS_MODE == 2)
// 注意：如果这里编译报类型错误，说明你当前库里的 UDP 类名不同
// 只需要按 zf_driver_udp.cpp 里的实际类名修改这一行
zf_driver_udp udp_dev;
#endif

uint8 *gray_image = NULL;

// 逐飞助手图像缓冲区
uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

// 逐飞助手三条 X 边界
uint8 x1_boundary[UVC_HEIGHT];
uint8 x2_boundary[UVC_HEIGHT];
uint8 x3_boundary[UVC_HEIGHT];

static volatile sig_atomic_t exit_requested = 0;
//发车标志
static int car_started = 0;

// ====================== TCP 收发包装函数 ======================
// 逐飞助手接口需要普通函数指针，所以这里封装 tcp_client_dev 的成员函数
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


// ====================== UDP 收发包装函数 ======================
// 逐飞助手接口需要普通函数指针，所以这里封装 udp_dev 的成员函数
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


// ====================== Ctrl+C 退出信号 ======================
void sigint_handler(int signum)
{
    exit_requested = 1;
}


// ====================== 停车保护 ======================
void stop_all(void)
{
    pit_timer.stop();

    drv8701e_pwm_1.set_duty(0);
    drv8701e_pwm_2.set_duty(0);

    printf("程序退出，电机已关闭。\r\n");
}


// ====================== main ======================
int main(int, char **)
{
    int car_started = 0;

    signal(SIGINT, sigint_handler);

    int frame_count = 0;
    auto last_print_time = std::chrono::steady_clock::now();


    // ====================== 1. 初始化图传通信 ======================
#if(IMAGE_TRANS_MODE == 1)

    // ====================== TCP 图传初始化 ======================
    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
        return -1;
    }

    // ====================== 初始化逐飞助手 TCP 通信接口 ======================
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

#elif(IMAGE_TRANS_MODE == 2)

    // ====================== UDP 图传初始化 ======================
    if(udp_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("udp ok\r\n");
    }
    else
    {
        printf("udp error\r\n");
        return -1;
    }

    // ====================== 初始化逐飞助手 UDP 通信接口 ======================
    seekfree_assistant_interface_init(udp_send_wrap, udp_read_wrap);

#else

    // ====================== 关闭图传 ======================
    printf("image transfer off\r\n");

#endif


    // ====================== 2. 配置逐飞助手图像信息 ======================
#if(IMAGE_TRANS_MODE != 0)

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        UVC_WIDTH,
        UVC_HEIGHT
    );

#endif


    // ====================== 3. 配置逐飞助手三条 X 边界 ======================
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


    // ====================== 5. 初始化电机信息 ======================
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);


    // 左右轮速度 PID
    pid_speed_init(car_config.speed_kp, car_config.speed_ki, car_config.speed_kd, 0, 0);

    // 中线差速 PID
    trackline_init(car_config.track_kp, car_config.track_ki, car_config.track_kd);
    


    // ====================== 7. 初始化摄像头 ======================
    if(uvc_dev.init(UVC_PATH) < 0)
    {
        printf("uvc init error\r\n");
        stop_all();
        return -1;
    }

    printf("uvc init ok\r\n");


    // ====================== 8. 启动速度闭环定时器 ======================
    pid_speed_start(20);


    // 主循环
    while(!exit_requested)
    {
        // 等待摄像头刷新
        if(uvc_dev.wait_image_refresh() < 0)
        {
            printf("camera refresh error\r\n");
            break;
        }

        // 获取摄像头灰度图指针
        gray_image = uvc_dev.get_gray_image_ptr();

        if(gray_image == NULL)
        {
            continue;
        }


        otsu_threshold(gray_image);


        
        image_process_from_bin_ptr(gray_image);
       
       
        
        static int display_count = 0;
        display_count++;

        if(display_count >= 5)
        {
            display_count = 0;
            display_gray_with_pid(gray_image);
        }


#if(IMAGE_TRANS_MODE != 0)

        // =====================================================
        // 4. 拷贝图像给逐飞助手
        // 此时 gray_image 已经是二值图
        // 上位机看到的是二值化后的图像
        // =====================================================
        memcpy(image_copy[0], gray_image, UVC_WIDTH * UVC_HEIGHT);


        // =====================================================
        // 5. 更新逐飞助手三条边界
        // x1 = 左线
        // x2 = 中线
        // x3 = 右线
        // =====================================================
    #if(INCLUDE_BOUNDARY_TYPE == 1)

        for(int i = 0; i < UVC_HEIGHT; i++)
        {
            x1_boundary[i] = l_border[i];
            x2_boundary[i] = center_line[i];
            x3_boundary[i] = r_border[i];
        }

    #endif


        // =====================================================
        // 6. 根据图传模式发送图像 + 边界到逐飞助手
        // 0：关闭图传
        // 1：TCP 图传
        // 2：UDP 图传
        // =====================================================
        seekfree_assistant_camera_send();

#endif


        Key_Process();

        int key = Key_GetValueOnce();

    if(key == KEY_4)
    {
        car_started = 1;

        std::cout << "KEY4 pressed, car start." << std::endl;
    }        

        trackline_refresh_wheel_targets(car_config.run_speed, UVC_HEIGHT - 45);
            pid_speed_set_target(
                trackline_wheel_target_right(),
                trackline_wheel_target_left()
            );
        }
    // ====================== 10. 退出保护 ======================
    stop_all();

    return 0;
}
