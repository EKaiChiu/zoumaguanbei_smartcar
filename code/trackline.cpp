#include "trackline.hpp"
#include "image.hpp"
#include "element.hpp"

// 左右轮目标速度
 int wheel_target_right = 0;
 int wheel_target_left  = 0;

// 巡线误差和转向输出
static float Err = 0.0f;
static float turn_output = 0.0f;


static float camera_P = 0.80f;
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
static float diff_gain = 5.0f;


// 方向修正
#define TRACK_DIR -1


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

/*
函数名称：static int get_speed_by_error(int base_speed, int abs_err)
功能说明：根据巡线误差绝对值无级调整运行速度，误差越大速度越低
参数说明：
    base_speed：基础速度
    abs_err：巡线误差绝对值
函数返回：调整后的运行速度
修改时间：2026年5月24日
备注：
    当误差小于 min_err 时，保持基础速度；
    当误差大于 max_err 时，降低到最低比例；
    中间区域按线性比例连续降速。
example：  get_speed_by_error(base_speed, abs_err);
 */
static int get_speed_by_error(int base_speed, int abs_err)
{
    /*
        小误差范围内不降速。
        这里对应你原来的 abs_err <= 8。
    */
    int min_err = 8;

    /*
        大误差时降到最低速度。
        这里对应你原来的 abs_err > 45。
    */
    int max_err = 45;

    /*
        最低速度比例。
        35 表示 base_speed 的 35%。
    */
    int min_percent = 30;

    /*
        最高速度比例。
        100 表示 base_speed 的 100%。
    */
    int max_percent = 100;

    abs_err = limit_int(abs_err, 0, max_err);

    int speed_percent = max_percent;

    if(abs_err <= min_err)
    {
        speed_percent = max_percent;
    }
    else if(abs_err >= max_err)
    {
        speed_percent = min_percent;
    }
    else
    {
        /*
            线性降速：
            abs_err 从 8 增加到 45 时，
            speed_percent 从 100 平滑降低到 35。
        */
        speed_percent = max_percent
                      - (abs_err - min_err) * (max_percent - min_percent) / (max_err - min_err);
    }

    int run_speed = base_speed * speed_percent / 100;

    return run_speed;
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
        return 4;
    }
    else if(y < image_h * 2 / 3)
    {
        return 10;
    }
    else if(y < image_h * 5 / 6)
    {
        return 20;
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


/*
函数名称：static float Err_Sum(int aim_y)
功能说明：根据近处中线、前方中线和中线斜率计算巡线误差，提高弯道转向力度
参数说明：
    aim_y：前瞻目标行，数值越大表示越靠近图像底部
函数返回：巡线误差，正值表示中线偏左，负值表示中线偏右
修改时间：2026年5月21日
备    注：
    near_error 用于保证车身贴近当前赛道中心
    far_error 用于提前观察前方弯道
    slope_error 用于发现中线倾斜趋势，弯道时会明显增大
example：  Err_Sum(UVC_HEIGHT - 30);
 */
static float Err_Sum(int aim_y)
{
    int image_mid = image_w / 2;

    int near_y = image_h - 12;
    int far_y = aim_y;

    near_y = limit_int(near_y, 0, image_h - 2);
    far_y = limit_int(far_y, 0, image_h - 2);

    if(Search_Stop_Line > 5 && Search_Stop_Line < image_h)
    {
        int visible_top = image_h - Search_Stop_Line;

        if(far_y < visible_top)
        {
            far_y = visible_top;
        }
    }

    int near_center = center_line[near_y];
    int far_center = center_line[far_y];

        // ===== 近处中线丢线保护 =====
    if(near_center <= border_min)
    {
        near_center = border_min; // 边沿保持：强制卡在最左侧边界，让近处误差拉满
    }
    else if(near_center >= border_max)
    {
        near_center = border_max; // 边沿保持：强制卡在最右侧边界，让近处误差拉满
    }

    // ===== 远处中线丢线保护 =====
    if(far_center <= border_min)
    {
        far_center = border_min;  // 边沿保持：远处误差拉满
    }
    else if(far_center >= border_max)
    {
        far_center = border_max;  // 边沿保持：远处误差拉满
    }
    float near_error = (float)(image_mid - near_center);
    float far_error = (float)(image_mid - far_center);

    float slope_error = (float)(near_center - far_center);

    float err = near_error * 0.40f
              + far_error * 0.40f
              + slope_error * 1.50f;

    return err;
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
    int dynamic_aim_y = aim_y;
    int last_abs_err = abs_int((int)Err); 
    if(last_abs_err > 20)
    {
        dynamic_aim_y = limit_int(aim_y + 20, 0, image_h - 15);
    }

    Err = Err_Sum(dynamic_aim_y);

    if(Err > -2.0f && Err < 2.0f)
    {
        Err = 0.0f;
    }

    turn_output = PD_Camera(0.0f, Err);

    int abs_err = abs_int((int)Err);
    int run_speed = base_speed;

    run_speed = get_speed_by_error(base_speed, abs_err);
    
    run_speed = limit_int(run_speed, 50, 300);

    float increase = -turn_output * diff_gain * TRACK_DIR;

    int diff = limit_int((int)increase, -800, 800);

    wheel_target_left  = run_speed - diff;
    wheel_target_right = run_speed + diff;

    wheel_target_left  = limit_int(wheel_target_left,  -800, 800);
    wheel_target_right = limit_int(wheel_target_right, -800, 800);
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