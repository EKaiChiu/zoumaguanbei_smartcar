#include "crossing.hpp"

//十字
volatile int Cross_Flaga=0;
volatile int Left_Down_Find=0; //十字使用，找到被置行数，没找到就是0
volatile int Left_Up_Find=0;   //四个拐点标志
volatile int Right_Down_Find=0;
volatile int Right_Up_Find=0;
uint8 mt9v03x_image_2[image_h][image_w];
uint8 mt9v03x_image_1[image_h][image_w];



 /*-------------------------------------------------------------------------------------------------------------------
   @brief     找上面的两个拐点，供十字使用
   @param     搜索的范围起点，终点
   @return    修改两个全局变量
              Left_Up_Find=0;
              Right_Up_Find=0;
   Sample     Find_Up_Point(int start,int end)
   @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
 -------------------------------------------------------------------------------------------------------------------*/
 void Find_Up_Point(int start,int end)
 {
     int i,t;
     Left_Up_Find=0;
     Right_Up_Find=0;
     if(start<end)
     {
         t=start;
         start=end;
         end=t;
     }
     if(end<=image_h-Search_Stop_Line)
         end=image_h-Search_Stop_Line;
     if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
         end=5;
     if(start>=image_h-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
         start=image_h-1-5;
     for(i=start;i>=end;i--)
     {
         if(Left_Up_Find==0&&//只找第一个符合条件的点
            abs(l_border[i]-l_border[i-1])<=5&&
            abs(l_border[i-1]-l_border[i-2])<=5&&
            abs(l_border[i-2]-l_border[i-3])<=5&&
               (l_border[i]-l_border[i+2])>=8&&
               (l_border[i]-l_border[i+3])>=15&&
               (l_border[i]-l_border[i+4])>=15)
         {
             Left_Up_Find=i;//获取行数即可
         }
         if(Right_Up_Find==0&&//只找第一个符合条件的点
            abs(r_border[i]-r_border[i-1])<=5&&//下面两行位置差不多
            abs(r_border[i-1]-r_border[i-2])<=5&&
            abs(r_border[i-2]-r_border[i-3])<=5&&
               (r_border[i]-r_border[i+2])<=-8&&
               (r_border[i]-r_border[i+3])<=-15&&
               (r_border[i]-r_border[i+4])<=-15)
         {
             Right_Up_Find=i;//获取行数即可
         }
         if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
         {
             break;
         }
     }
     if(abs(Right_Up_Find-Left_Up_Find)>=30)//纵向撕裂过大，视为误判
     {
         Right_Up_Find=0;
         Left_Up_Find=0;
     }
 }

 /*-------------------------------------------------------------------------------------------------------------------
   @brief     找下面的两个拐点，供十字使用
   @param     搜索的范围起点，终点
   @return    修改两个全局变量
              Right_Down_Find=0;
              Left_Down_Find=0;
   Sample     Find_Down_Point(int start,int end)
   @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
 -------------------------------------------------------------------------------------------------------------------*/
 void Find_Down_Point(int start,int end)
 {
     int i,t;
     Right_Down_Find=0;
     Left_Down_Find=0;
     if(start<end)
     {
         t=start;
         start=end;
         end=t;
     }
     if(start>=image_h-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
         start=image_h-1-5;
     if(end<=image_h-Search_Stop_Line)
         end=image_h-Search_Stop_Line;
     if(end<=5)
        end=5;
     for(i=start;i>=end;i--)
     {
         if(Left_Down_Find==0&&//只找第一个符合条件的点
            abs(l_border[i]-l_border[i+1])<=5&&//角点的阈值可以更改
            abs(l_border[i+1]-l_border[i+2])<=5&&
            abs(l_border[i+2]-l_border[i+3])<=5&&
               (l_border[i]-l_border[i-2])>=8&&
               (l_border[i]-l_border[i-3])>=15&&
               (l_border[i]-l_border[i-4])>=15)
         {
             Left_Down_Find=i;//获取行数即可
         }
         if(Right_Down_Find==0&&//只找第一个符合条件的点
            abs(r_border[i]-r_border[i+1])<=5&&//角点的阈值可以更改
            abs(r_border[i+1]-r_border[i+2])<=5&&
            abs(r_border[i+2]-r_border[i+3])<=5&&
               (r_border[i]-r_border[i-2])<=-8&&
               (r_border[i]-r_border[i-3])<=-15&&
               (r_border[i]-r_border[i-4])<=-15)
         {
             Right_Down_Find=i;
         }
         if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
         {
             break;
         }
     }
 }

 /*-------------------------------------------------------------------------------------------------------------------
   @brief     左补线
   @param     补线的起点，终点
   @return    null
   Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
   @note      补的直接是边界，点最好是可信度高的,不要乱补
 -------------------------------------------------------------------------------------------------------------------*/
 void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
 {
     int i,max,a1,a2;
     int hx;
     if(x1>=image_w-1)//起始点位置校正，排除数组越界的可能
        x1=image_w-1;
     else if(x1<=0)
         x1=0;
      if(y1>=image_h-1)
         y1=image_h-1;
      else if(y1<=0)
         y1=0;
      if(x2>=image_w-1)
         x2=image_w-1;
      else if(x2<=0)
              x2=0;
      if(y2>=image_h-1)
         y2=image_h-1;
      else if(y2<=0)
              y2=0;
     a1=y1;
     a2=y2;
     if(a1>a2)//坐标互换
     {
         max=a1;
         a1=a2;
         a2=max;
     }
     for(i=a1;i<=a2;i++)//根据斜率补线即可
     {
         hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
         if(hx>=image_w)
             hx=image_w;
         else if(hx<=0)
             hx=0;
         l_border[i]=hx;
     }
 }

 /*-------------------------------------------------------------------------------------------------------------------
   @brief     右补线
   @param     补线的起点，终点
   @return    null
   Sample     Right_Add_Line(int x1,int y1,int x2,int y2);
   @note      补的直接是边界，点最好是可信度高的，不要乱补
 -------------------------------------------------------------------------------------------------------------------*/
 void Right_Add_Line(int x1,int y1,int x2,int y2)//右补线,补的是边界
 {
     int i,max,a1,a2;
     int hx;
     if(x1>=image_w-1)//起始点位置校正，排除数组越界的可能
        x1=image_w-1;
     else if(x1<=0)
         x1=0;
     if(y1>=image_h-1)
         y1=image_h-1;
     else if(y1<=0)
         y1=0;
     if(x2>=image_w-1)
         x2=image_w-1;
     else if(x2<=0)
         x2=0;
     if(y2>=image_h-1)
         y2=image_h-1;
     else if(y2<=0)
          y2=0;
     a1=y1;
     a2=y2;
     if(a1>a2)//坐标互换
     {
         max=a1;
         a1=a2;
         a2=max;
     }
     for(i=a1;i<=a2;i++)//根据斜率补线即可
     {
         hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
         if(hx>=image_w)
             hx=image_w;
         else if(hx<=0)
             hx=0;
         r_border[i]=hx;
     }
 }

 /*-------------------------------------------------------------------------------------------------------------------
   @brief     左边界延长
   @param     延长起始行数，延长到某行
   @return    null
   Sample     Stop_Detect(void)
   @note      从起始点向上找5个点，算出斜率，向下延长，直至结束点
 -------------------------------------------------------------------------------------------------------------------*/
 void Lengthen_Left_Boundry(int start,int end)
 {
     int i,t;
     float k=0;
     if(start>=image_h-1)//起始点位置校正，排除数组越界的可能
         start=image_h-1;
     else if(start<=0)
         start=0;
     if(end>=image_h-1)
         end=image_h-1;
     else if(end<=0)
         end=0;
     if(end<start)//++访问，坐标互换
     {
         t=end;
         end=start;
         start=t;
     }

     if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
     {
          Left_Add_Line(l_border[start],start,l_border[end],end);
     }

     else
     {
         k=(float)(l_border[start]-l_border[start-4])/5.0;//这里的k是1/斜率
         for(i=start;i<=end;i++)
         {
             l_border[i]=(int)(i-start)*k+l_border[start];//(x=(y-y1)*k+x1),点斜式变形
             if(l_border[i]>=image_w-1)
             {
                 l_border[i]=image_w-1;
             }
             else if(l_border[i]<=0)
             {
                 l_border[i]=0;
             }
         }
     }
 }

 /*-------------------------------------------------------------------------------------------------------------------
   @brief     右左边界延长
   @param     延长起始行数，延长到某行
   @return    null
   Sample     Stop_Detect(void)
   @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
 -------------------------------------------------------------------------------------------------------------------*/
 void Lengthen_Right_Boundry(int start,int end)
 {
     int i,t;
     float k=0;
     if(start>=image_h-1)//起始点位置校正，排除数组越界的可能
         start=image_h-1;
     else if(start<=0)
         start=0;
     if(end>=image_h-1)
         end=image_h-1;
     else if(end<=0)
         end=0;
     if(end<start)//++访问，坐标互换
     {
         t=end;
         end=start;
         start=t;
     }

     if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
     {
         Right_Add_Line(r_border[start],start,r_border[end],end);
     }
     else
     {
         k=(float)(r_border[start]-r_border[start-4])/5.0;//这里的k是1/斜率
         for(i=start;i<=end;i++)
         {
             r_border[i]=(int)(i-start)*k+r_border[start];//(x=(y-y1)*k+x1),点斜式变形
             if(r_border[i]>=image_w-1)
             {
                 r_border[i]=image_w-1;
             }
             else if(r_border[i]<=0)
             {
                 r_border[i]=0;
             }
         }
     }
 }


/*-------------------------------------------------------------------------------------------------------------------
   @brief     十字检测
   @param     null
   @return    null
   Sample     Cross_Detect(void);
   @note      利用四个拐点判别函数，查找四个角点，根据找到拐点的个数决定是否补线
 -------------------------------------------------------------------------------------------------------------------*/
 void Cross_Detect()
 {
     int down_search_start=0;//下点搜索开始行
     Cross_Flaga=0;
     
         Left_Up_Find=0;
         Right_Up_Find=0;
         if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
         {
             Find_Up_Point( image_h-1, 0 );
             if(Left_Up_Find==0&&Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
             {
                 return;
             }
         }
         if(Left_Up_Find!=0&&Right_Up_Find!=0)//找到两个上点，就找到十字了
         {
             Cross_Flaga=1;//对应标志位，便于各元素互斥掉
             down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
             Find_Down_Point(image_h-5,down_search_start+2);//在上拐点下2行作为下点的截止行
             if(Left_Down_Find<=Left_Up_Find)
             {
                 Left_Down_Find=0;//下点不可能比上点还靠上
             }
             if(Right_Down_Find<=Right_Up_Find)
             {
                 Right_Down_Find=0;//下点不可能比上点还靠上
             }
             if(Left_Down_Find!=0&&Right_Down_Find!=0)
             {//四个点都在，无脑连线，这种情况显然很少
                 Left_Add_Line (l_border[Left_Up_Find ],Left_Up_Find ,l_border[Left_Down_Find ] ,Left_Down_Find);
                 Right_Add_Line(r_border[Right_Up_Find],Right_Up_Find,r_border[Right_Down_Find],Right_Down_Find);
             }
             else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//这里使用的都是斜率补线
             {//三个点                                     //01
                 Lengthen_Left_Boundry(Left_Up_Find-1,image_h-1);
                 Right_Add_Line(r_border[Right_Up_Find],Right_Up_Find,r_border[Right_Down_Find],Right_Down_Find);
             }
             else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
             {//三个点                                     //10
                 Left_Add_Line (l_border[Left_Up_Find ],Left_Up_Find ,l_border[Left_Down_Find ] ,Left_Down_Find);
                 Lengthen_Right_Boundry(Right_Up_Find-1,image_h-1);
             }
             else if(Left_Down_Find==0&&Right_Down_Find==0)//11
             {//就俩上点                                   //00
                 Lengthen_Left_Boundry (Left_Up_Find-1,image_h-1);
                 Lengthen_Right_Boundry(Right_Up_Find-1,image_h-1);
             }
         }
         else
         {
             Cross_Flaga=0;
         }
     
     //角点相关变量，debug使用
     //ips200_showuint8(0,12,Cross_Flag);
 //    ips200_showuint8(0,13,Island_State);
 //    ips200_showuint8(50,12,Left_Up_Find);
 //    ips200_showuint8(100,12,Right_Up_Find);
 //    ips200_showuint8(50,13,Left_Down_Find);
 //    ips200_showuint8(100,13,Right_Down_Find);
 }