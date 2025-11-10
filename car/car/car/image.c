
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "image.h"









/*----------------------------------------------全局变量-------------------------------------------------------*/

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];           // 原始灰度图
uint8 mt9v03x_imagebin[MT9V03X_H][MT9V03X_W];        // 二值化图像
uint8 mid_line[MT9V03X_H];                           // 存储每一行的中线位置（视觉顺序）
uint8 left_line[MT9V03X_H];                           // 存储每一行的左边界位置（视觉顺序）
uint8 right_line[MT9V03X_H];                           // 存储每一行的右边界位置（视觉顺序）
uint8 carmid_line[MT9V03X_H];       //小车真正的中线


uint8 lossline_threshold_rectan = 30;       //直角上方无线的阈值
uint8 lossline_threshold_rectanless = 5;       //直角上方无线相差的阈值








/*----------------------------------------------函数实现-------------------------------------------------------*/

// 二值化函数
void binarizeImages(uint8 image[MT9V03X_H][MT9V03X_W], uint8 imagebin[MT9V03X_H][MT9V03X_W], int threshold)
{
    for (int r = 0; r < MT9V03X_H; ++r) {
        for (int c = 0; c < MT9V03X_W; ++c) {
            imagebin[r][c] = (image[r][c] > threshold) ? 255 : 0;
        }
    }
}

// Otsu 算法计算最佳阈值
uint8 otsuThreshold(uint8* image) {
    const int GrayScale = 256;
    int Pixel_Max = 0;
    int Pixel_Min = 255;
    uint16 width = MT9V03X_W;
    uint16 height = MT9V03X_H;
    uint32 gray_sum = 0;
    uint8 threshold = 0;
    uint8* data = image;

    // 动态分配避免栈溢出
    int* pixelCount = (int*)calloc(GrayScale, sizeof(int));
    float* pixelPro = (float*)calloc(GrayScale, sizeof(float));

    if (!pixelCount || !pixelPro) {
        printf("内存分配失败\n");
        free(pixelCount);
        free(pixelPro);
        return 60;
    }

    // 统计像素分布（采样间隔为 2 提高效率）
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            int index = data[i * width + j];
            pixelCount[index]++;
            gray_sum += index;
            if (index > Pixel_Max) Pixel_Max = index;
            if (index < Pixel_Min) Pixel_Min = index;
        }
    }

    float total = (height / 2) * (width / 2);  // 采样点总数
    for (int i = Pixel_Min; i <= Pixel_Max; i++) {
        pixelPro[i] = (float)pixelCount[i] / total;
    }

    // Otsu 公式计算最大类间方差
    float w0 = 0.0f, u0tmp = 0.0f, deltaTmp = 0.0f, deltaMax = 0.0f;
    float u0 = 0.0f, u1 = 0.0f, w1, u1tmp;

    for (int j = Pixel_Min; j <= Pixel_Max; j++) {
        w0 += pixelPro[j];
        u0tmp += j * pixelPro[j];

        w1 = 1.0f - w0;
        if (w1 == 0.0f) break;

        u1tmp = ((float)gray_sum / total) - u0tmp;
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;

        deltaTmp = w0 * w1 * (u0 - u1) * (u0 - u1);

        if (deltaTmp > deltaMax) {
            deltaMax = deltaTmp;
            threshold = j;
        }
    }

    free(pixelCount);
    free(pixelPro);

    return threshold;
}

// 赛道中线识别
void find_lane_midline(void) {


    // 初始化左右边界为图像中间附近的合理值
    uint8 prev_left = MAX(0, (MT9V03X_W / 2) - SEARCH_RANGE);
    uint8 prev_right = MIN(MT9V03X_W - 1, (MT9V03X_W / 2) + SEARCH_RANGE);
    uint8 prev_mid = (prev_left + prev_right) / 2;

    uint8 left_count = 0;               //存储丢失左边界的行数
    uint8 right_count = 0;              //存储丢失左边界的行数

    //每用一次此函数，均会清零这四个值
    uint8 left_flag = 0;               //存储丢失左边界的标志
    uint8 right_flag = 0;              //存储丢失左边界的标志


    // 遍历图像的每一行（从下到上）
    for (int sensor_row = MT9V03X_H - 1; sensor_row >= 0; sensor_row--) {
        int visual_row = sensor_row;

        uint8 current_left = prev_left;  // 默认使用上一次的左边界
        uint8 current_right = prev_right; // 默认使用上一次的右边界

        // 左边界搜索：向右找第一个白点，且右相邻为白，左相邻为黑
        for (int col = MAX(1, prev_mid - SEARCH_RANGE);
            col <= MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col++) {

                left_flag = 0;//判断每行时都要清零
                right_flag = 0;//判断每行时都要清零

            if (mt9v03x_imagebin[sensor_row][col - 1] == 0 &&
                mt9v03x_imagebin[sensor_row][col] == 255 &&
                mt9v03x_imagebin[sensor_row][col + 1] == 255) {

                current_left = col;
                prev_left = col;  // 找到新左边界，更新缓存
                left_count = 0;
                left_flag = 1;
                break;
            }
        }

        // 如果未找到左边界，计数器加一
        if (left_flag == 0) {
            left_count++;
        }

        // 右边界搜索：向左找第一个白点，且左相邻为白，右相邻为黑
        for (int col = MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col >= MAX(1, prev_mid - SEARCH_RANGE);
            col--) {

            if (mt9v03x_imagebin[sensor_row][col - 1] == 255 &&
                mt9v03x_imagebin[sensor_row][col] == 255 &&
                mt9v03x_imagebin[sensor_row][col + 1] == 0) {

                current_right = col;
                prev_right = col;  // 找到新右边界，更新缓存
                right_count = 0;
                right_flag = 1;
                break;
            }
        }

        // 如果未找到右边界，计数器加一
        if (right_flag == 0) {
            right_count++;
        }
        // ✅ 每一行都更新中线、左边界、右边界数组
        left_line[visual_row] = current_left;
        right_line[visual_row] = current_right;
        mid_line[visual_row] = (current_left + current_right) / 2;

        // 基于当前左右边界计算中线
        prev_mid = (current_left + current_right) / 2;


    }

    /*-----------------------------打印right_count和left_count值-----------------------------*/
    printf("\n打印right_count和left_count值\n");
    printf("right_count = %d, left_count = %d\n", right_count, left_count);
    printf("\n");



 /*----------------------------------------------起点预处理-------------------------------------------------------*/
    
    static uint8 start_times = 0; //经过起始点次数
    uint8 lstart = 0;   //满足条件的start所在的行
    uint8 rstart = 0;   //满足条件的start所在的行
    uint8 lstart_count = 0; //满足的点
    uint8 rstart_count = 0; //满足的点
    uint8 lstart_flag = 0;//满足标志位
    uint8 rstart_flag = 0;//满足标志位


    // 检查left_line左边是否有至少4个点大于当前点
    for (int i = 4; i <= MT9V03X_H - 5; i++) {
    for (int j = 0; j < i; j++) {
        if (left_line[j]-left_line[i]> 6) {
            lstart_count++;
            if (lstart_count >= 4) {
                lstart = i;
                lstart_flag = 1;
                break;
            }

        }
    }
    // 检查right_line左边是否有至少4个点小于当前点
    for (int j = 0; j < i; j++) {
        if (right_line[i] - right_line[j] > 6) {
            rstart_count++;
            if (rstart_count >= 4) {
                rstart = i;
                rstart_flag = 1;
                break;
            }

        }
    }
}
    

    uint8 startdiffer_count = 0;
    uint8 startdiffer_flag = 0;


    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        for (int j = i; j < MT9V03X_H-1; j++) {

            if (right_line[j] - left_line[j] > 70&&(abs((right_line[j] - left_line[j]) - (right_line[j + 1] - left_line[j + 1])) ==0)) {
                startdiffer_count++;
                if (startdiffer_count >= 6) {
                    startdiffer_flag = 1;
                    break;
                }

            }


        }
        
        if (startdiffer_flag == 1) break;
    }

    

    /*----------------------------------------------起点的判断-------------------------------------------------------*/
  if (startdiffer_flag==1&&lstart_flag == 1&& rstart_flag==1&&(lstart > 90))
  {
      start_times++;

  }

  /*-----------------------------打印start_times值-----------------------------*/
  printf("\n打印start_times值\n");
  printf("start_times = %d", start_times);
  printf("\n");
  printf("\n");







    /*----------------------------------------------左环岛预处理-------------------------------------------------------*/

    uint8 lleft4 = 0;  //左边计数点满足标志
    uint8 lright4 = 0; //右边计数点满足标志
    uint8 lleft_count = 0;//左边计数点
    uint8 lright_count = 0;//右边计数点

    uint8 rleft4 = 0;  //左边计数点满足标志
    uint8 rright4 = 0; //右边计数点满足标志
    uint8 rleft_count = 0;//左边计数点
    uint8 rright_count = 0;//右边计数点

    uint8 leftcircle_flag = 0;            //左环岛标志
    uint8 rightcircle_flag = 0;          //右环岛标志




    for (int i = 4; i <= MT9V03X_H - 5; i++) {


        // 检查左边是否有至少4个点小于当前点
        for (int j = 0; j < i; j++) {
            if ( left_line[i]-left_line[j] >5) {
                lleft_count++;
                if (lleft_count >= 4) {
                    lleft4 = 1;
                    break;
                }
                   
            }
        }
        // 检查右边是否有至少4个点小于当前点
        for (int j = i + 1; j < MT9V03X_H; j++) {
            if (left_line[i] - left_line[j] > 5) {
                lright_count++;
                if (lright_count >= 4) {
                    lright4 = 1;
                    break;
                }
            }
        }
        if (lleft4 && lright4 == 1) {
            leftcircle_flag = 1;
        }

    }


    /*----------------------------------------------右环岛预处理-------------------------------------------------------*/
    for (int i = 4; i <= MT9V03X_H - 5; i++) {



        // 检查左边是否有至少4个点大于当前点
        for (int j = 0; j < i; j++) {
            if (right_line[j] - right_line[i]>5) {
                rleft_count++;
                if (rleft_count >= 4) {
                    rleft4 = 1;
                    break;
                }

            }
        }
        // 检查右边是否有至少4个点大于当前点
        for (int j = i + 1; j < MT9V03X_H; j++) {
            if (right_line[j] -right_line[i]>5) {
                rright_count++;
                if (rright_count >= 4) {
                    rright4 = 1;
                    break;
                }
            }
        }
        if (rleft4 && rright4 == 1) {
            rightcircle_flag = 1;
        }

    }



    /*----------------------------------------------出环岛预处理-------------------------------------------------------*/
    uint8 outcricle_lcount=0;//满足的点
    uint8 outcricle_rcount=0;//满足的点

    uint8 outcricle_lflag=0;
    uint8 outcricle_rflag=0;

    uint8 outcricle_flag=0;//出环岛标志位


        for (int i = MT9V03X_H - 1; i >= 6; i--) {

        // 检查右侧线的连续6行条件
        for (int j = i; j < MT9V03X_H; j++) {
            if (abs(right_line[j] - right_line[i]) <2) {
                outcricle_rcount++;
                if (outcricle_rcount >= 6) {
                    outcricle_rflag = 1;
                    break;
                }

            }
        }
        // 检查左边是否有至少4个点大于当前点
        for (int j = i ; j < MIN(i+10,MT9V03X_H); j++) {
            if (left_line[j] - left_line[i] > 5&& left_line[j]!=left_line[j+1]) {
                outcricle_lcount++;
                if (outcricle_lcount >= 6) {
                    outcricle_lflag = 1;
                    break;
                }
            }
        }
        if (outcricle_rflag && outcricle_lflag == 1) {
            outcricle_flag = 1;
        }

    }
   





    /*----------------------------------------------对中线根据元素特征进行处理-------------------------------------------------------*/


            /*----------------------------------------------出环岛轨道-------------------------------------------------------*/
        if (outcricle_flag == 1)
        {
            // 逐个元素复制
            for (int i = 0; i < MT9V03X_H; i++) {
                carmid_line[i] = right_line[i];
            }

        }

    /*----------------------------------------------直轨道，十字路口，折线-------------------------------------------------------*/
    if (right_count < 2 && left_count < 2)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = mid_line[i];
        }
    }

    /*----------------------------------------------右直角轨道-------------------------------------------------------*/
    if (((right_count > lossline_threshold_rectan )|| (left_count > lossline_threshold_rectan))
        && ((right_count - left_count) > lossline_threshold_rectanless))
    {
        // 前 right_count 个元素赋值为 MT9V03X_W
        for (int i = 0; i < right_count; i++) {
            carmid_line[i] = MT9V03X_W;
        }

        // 剩余元素复制自 mid_line
        for (int i = right_count; i < MT9V03X_H; i++) {
            carmid_line[i] = mid_line[i];
        }

    }

    /*----------------------------------------------左直角轨道-------------------------------------------------------*/
    if (((right_count > lossline_threshold_rectan) || (left_count > lossline_threshold_rectan))
        && ((left_count - right_count) > lossline_threshold_rectanless))
    {
        // 前 right_count 个元素赋值为 MT9V03X_W
        for (int i = 0; i < left_count; i++) {
            carmid_line[i] = 0;
        }

        // 剩余元素复制自 mid_line
        for (int i = left_count; i < MT9V03X_H; i++) {
            carmid_line[i] = mid_line[i];
        }

    }




    /*----------------------------------------------左环岛轨道-------------------------------------------------------*/
    if (leftcircle_flag == 1&& rightcircle_flag==0)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = left_line[i];
        }

    }

    /*----------------------------------------------右环岛轨道-------------------------------------------------------*/
    if (rightcircle_flag == 1&& leftcircle_flag == 0)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = right_line[i];
        }

    }







    /*----------------------------------------------虚线轨道判断-------------------------------------------------------*/
    uint8 dash_flag = 0;//虚线轨道判断标志位
    uint8 dash_differ = (right_count > left_count) ? (right_count - left_count) : (left_count - right_count);
    if (right_count > 70&& left_count>70&& dash_differ<3)
    {
        dash_flag = 1;

    }

    /*-----------------------------打印dash_flag值-----------------------------*/
    printf("\n打印dash_flag值\n");
    printf("dash_flag = %d", dash_flag);
    printf("\n");
    printf("\n");




    /*-----------------------------转角值的处理并求图片的差值-----------------------------*/
    uint32 image_differ = 0;
    int image_differsum = 0;
    uint32 image_changshu = 93 * 120;
    for (int i = 0; i < MT9V03X_H; i++) {
        image_differ += carmid_line[i];
    }
    image_differsum = image_differ - image_changshu;

    /*-----------------------------打印图片的差值-----------------------------*/
    printf("\n打印image_differsum值\n");
    printf("image_differsum = %d", image_differsum);
    printf("\n");
    printf("\n");











   


    






    












}

























