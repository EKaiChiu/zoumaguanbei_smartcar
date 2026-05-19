#include "trackline.hpp"
#include "image.hpp"

// 左右轮目标速度
static int wheel_target_right = 0;
static int wheel_target_left  = 0;

// 巡线误差和转向输出
static float Err = 0.0f;
static float turn_output = 0.0f;


static float camera_P = 0.41f;
static float camera_D = 0.01f;

static float error_last = 0.0f;


// =====================================================
// 输出滤波缓存
// 对应原代码 Pre1_Error[4]
// =====================================================
static float Pre1_Error[4] = {0, 0, 0, 0};


// =====================================================
// 差速放大系数
// 转向力度不够就加大，比如 1.5 / 2.0 / 2.5
// =====================================================
static float diff_gain = 1.0f;


// 方向修正
#define TRACK_DIR 1


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


static int abs_int(int value)
{
    return value >= 0 ? value : -value;
}


// =====================================================
// 行权重
// =====================================================
static int get_weight_by_y(int y)
{
    y = limit_int(y, 0, image_h - 1);

    /*
        y 越大，越靠近图像底部。
        这里主要看中下部。
    */
    if(y < image_h * 1 / 3)
    {
        return 1;
    }
    else if(y < image_h * 1 / 2)
    {
        return 2;
    }
    else if(y < image_h * 2 / 3)
    {
        return 8;
    }
    else if(y < image_h * 5 / 6)
    {
        return 16;
    }
    else
    {
        return 6;
    }
}


// =====================================================
// 初始化 trackline
//
// kp 对应原代码 P
// ki 暂时不用
// kd 对应原代码 D
// =====================================================
void trackline_init(float kp, float ki, float kd)
{
    (void)ki;

    camera_P = kp;
    camera_D = kd;

    Err = 0.0f;
    turn_output = 0.0f;
    error_last = 0.0f;

    wheel_target_right = 0;
    wheel_target_left = 0;

    Pre1_Error[0] = 0.0f;
    Pre1_Error[1] = 0.0f;
    Pre1_Error[2] = 0.0f;
    Pre1_Error[3] = 0.0f;
}


// =====================================================
// 计算加权误差
// err += (image_w / 2 - center_line[i]) * weight;
// =====================================================
static float Err_Sum(int aim_y)
{
    float err = 0.0f;
    float weight_count = 0.0f;

    int image_mid = image_w / 2;

    int start_y = image_h - 2;
    int end_y = aim_y;

    end_y = limit_int(end_y, 0, image_h - 2);

    /*
        如果 Search_Stop_Line 有效，就不要使用看不见的区域。
        Search_Stop_Line 表示从底部向上可见的有效白列高度。
    */
    if(Search_Stop_Line > 5 && Search_Stop_Line < image_h)
    {
        int visible_top = image_h - Search_Stop_Line;

        if(visible_top > end_y)
        {
            end_y = visible_top;
        }
    }

    for(int y = start_y; y >= end_y; y--)
    {
        int center = center_line[y];

        /*
            中线异常就跳过
        */
        if(center <= border_min || center >= border_max)
        {
            continue;
        }

        int weight = get_weight_by_y(y);

        err += (float)(image_mid - center) * weight;
        weight_count += weight;
    }

    if(weight_count <= 0.0f)
    {
        return Err;
    }

    return err / weight_count;
}


// =====================================================
// 摄像头 PD
// =====================================================
static float PD_Camera(float expect_val, float err)
{
    float error_current = err - expect_val;
    float error_delta = error_current - error_last;

    float u = camera_P * error_current + camera_D * error_delta;

    error_last = error_current;

    /*
        简单滤波，防止输出突变太大
    */
    Pre1_Error[3] = Pre1_Error[2];
    Pre1_Error[2] = Pre1_Error[1];
    Pre1_Error[1] = Pre1_Error[0];
    Pre1_Error[0] = u;

    float pwm_turn =
          Pre1_Error[0] * 0.80f
        + Pre1_Error[1] * 0.10f
        + Pre1_Error[2] * 0.06f
        + Pre1_Error[3] * 0.04f;

    return pwm_turn;
}


// =====================================================
// 根据中线刷新左右轮目标速度
// =====================================================
void trackline_refresh_wheel_targets(int base_speed, int aim_y)
{
    /*
        1. 计算多行加权误差
        Err > 0：中线在图像左侧，需要左转
        Err < 0：中线在图像右侧，需要右转
    */
    Err = Err_Sum(aim_y);

    /*
        2. 小误差死区，防止直道左右抖
    */
    if(Err > -2.0f && Err < 2.0f)
    {
        Err = 0.0f;
    }

    /*
        3. 摄像头 PD 输出
    */
    turn_output = PD_Camera(0.0f, Err);

    /*
        4. 误差越大，基础速度越低
    */
    int abs_err = abs_int((int)Err);

    int run_speed = base_speed;

    if(abs_err > 35)
    {
        run_speed = base_speed * 55 / 100;
    }
    else if(abs_err > 25)
    {
        run_speed = base_speed * 65 / 100;
    }
    else if(abs_err > 15)
    {
        run_speed = base_speed * 80 / 100;
    }
    else if(abs_err > 8)
    {
        run_speed = base_speed * 90 / 100;
    }

    run_speed = limit_int(run_speed, 80, 300);

    /*
        5. 差速量
    */
    float increase = -turn_output * diff_gain * TRACK_DIR;

    /*
        6. 差速限幅
    */
    int diff = limit_int((int)increase, -220, 220);

    /*
        7. 生成左右轮目标速度
    */
    wheel_target_left  = run_speed - diff;
    wheel_target_right = run_speed + diff;

    /*
        8. 限幅
    */
    wheel_target_left  = limit_int(wheel_target_left,  0, 600);
    wheel_target_right = limit_int(wheel_target_right, 0, 600);
}


// =====================================================
// 获取右轮目标速度
// =====================================================
int trackline_wheel_target_right(void)
{
    return wheel_target_right;
}


// =====================================================
// 获取左轮目标速度
// =====================================================
int trackline_wheel_target_left(void)
{
    return wheel_target_left;
}


// =====================================================
// 获取当前误差，方便显示调试
// =====================================================
float trackline_get_error(void)
{
    return Err;
}


// =====================================================
// 获取当前转向输出，方便显示调试
// =====================================================
float trackline_get_turn_output(void)
{
    return turn_output;
}