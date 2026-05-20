#include "image.hpp"
#include "element.hpp"

// =====================================================
// 全局数组定义
// =====================================================
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

int Boundry_Start_Left = 0;//左边界首次有效的行号
int Boundry_Start_Right = 0;//右边界首次有效的行号

int Left_Lost_Flag[image_h] = {0};  //每行左边界是否丢失
int Right_Lost_Flag[image_h] = {0}; //每行右边界是否丢失


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
函数返回：绝对值
修改时间：2026年5月20日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

// =====================================================
// 拷贝一维二值图到二维 bin_image
// =====================================================
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


// =====================================================
// 给图像加黑框，防止扫描越界
// =====================================================
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


// =====================================================
// 初始化所有边界数据
// =====================================================
static void clear_line_data(void)
{
    Search_Stop_Line = 0;

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
修改时间：2026年5月20日
example：  get_standard_road_width(y);
 */
static int get_standard_road_width(int y)
{
    y = limit_int(y, 0, image_h - 1);

    int top_width = image_w / 3;
    int bottom_width = image_w * 3 / 4;

    int width = top_width + (bottom_width - top_width) * y / (image_h - 1);

    return width;
}

/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2026年5月20日
备    注：
example：  get_start_point(image_h-2)
*/
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
	uint8 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                        uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[image_w]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                    特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2026年5月20日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
*/

#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

//存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},	     {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},	     {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        //特殊情况
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
        && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2)
        {
            printf("\n左右相遇退出\n");	
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
        	
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}

/*
函数名称:void get_right(uint16 total_R)
        void get_left(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2026年5月20日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h - 2;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		l_border[i] = border_min;//左边线初始化放到最左边，右边线放到最右边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h - 2;
	//左边
	for (j = 0; j < total_L; j++)
	{
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0] + 1;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{

				bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{

				bin_image[i][j] = 0;//黑

			}

		}
	}

}

/*
函数名称：void get_center_line(uint8 hightest)
功能说明：根据左右边线提取中线，支持单边丢线时根据标准赛道宽度补中线
参数说明：
    hightest：八邻域搜索到的最高行，数值越小表示搜索得越远
函数返回：无
修改时间：2026年5月20日
example：  get_center_line(hightest);
 */
void get_center_line(uint8 hightest)
{
    int last_center = image_w / 2;

    hightest = limit_int(hightest, 0, image_h - 2);

    Left_Lost_Time = 0;
    Right_Lost_Time = 0;
    Both_Lost_Time = 0;

    for(int y = 0; y < image_h; y++)
    {
        center_line[y] = image_w / 2;
        Road_Wide[y] = border_max - border_min;
        Left_Lost_Flag[y] = 1;
        Right_Lost_Flag[y] = 1;
    }

    for(int y = hightest; y < image_h - 1; y++)
    {
        int left = l_border[y];
        int right = r_border[y];

        int left_valid = 1;
        int right_valid = 1;

        if(left <= border_min + 1)
        {
            left_valid = 0;
        }

        if(right >= border_max - 1)
        {
            right_valid = 0;
        }

        if(right <= left)
        {
            left_valid = 0;
            right_valid = 0;
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

        int half_width = get_standard_road_width(y) / 2;

        if(left_valid && right_valid)
        {
            center_line[y] = (left + right) >> 1;
            Road_Wide[y] = right - left;
        }
        else if(left_valid && !right_valid)
        {
            center_line[y] = left + half_width;
            Road_Wide[y] = get_standard_road_width(y);
        }
        else if(!left_valid && right_valid)
        {
            center_line[y] = right - half_width;
            Road_Wide[y] = get_standard_road_width(y);
        }
        else
        {
            center_line[y] = last_center;
            Road_Wide[y] = get_standard_road_width(y);
        }

        center_line[y] = limit_int(center_line[y], border_min, border_max);
        last_center = center_line[y];
    }

    Search_Stop_Line = image_h - hightest;
}

/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2026年5月20日
备    注：
example： image_process();
 */
void image_process(void)
{
uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
image_filter(bin_image);//滤波
image_draw_black_rect();//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
	printf("正在开始八领域\n");
	search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	printf("八邻域已结束\n");
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
    get_center_line(hightest);//获取中线

    element_process();//元素识别

    get_center_line(hightest);//重新生成中线
	//处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}
}

// =====================================================
// 对外主入口
// bin_src 必须已经是二值图
// main.cpp 里应先调用 camera.cpp 的 otsu_threshold()
// =====================================================
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


// =====================================================
// 兼容旧名字
// =====================================================
void image_process_by_white_column(uint8 *bin_src)
{
    image_process_from_bin_ptr(bin_src);
}