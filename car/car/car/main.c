#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
//#include "image.h"
#include "imageee.h"
#include "pid.h"



// stb_image 支持 PNG 图像加载/写入
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STBI_WINDOWS_UTF8
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


#define CHANNELS      3        // RGB 三通道


/*----------------------------------------------主函数-------------------------------------------------------*/
int main(void) {
    int width, height, channels;
    uint8* img;

    // 加载图像并转为灰度图（单通道）
    img = stbi_load("a.png", &width, &height, &channels, 1);
    if (!img) {
        printf("无法加载图像，请确认路径和格式是否正确。\n");
        return -1;
    }

    printf("原图尺寸: %d x %d\n", width, height);
    printf("通道数: %d\n", channels);

    // 检查图像大小
    if (width != MT9V03X_W || height != MT9V03X_H) {
        printf("错误：图像尺寸必须是 %dx%d\n", MT9V03X_W, MT9V03X_H);
        stbi_image_free(img);
        return -1;
    }

    // 将图像复制进二维数组 mt9v03x_image（视觉顺序）
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < MT9V03X_W; j++) {
            mt9v03x_image[i][j] = img[i * MT9V03X_W + j];
        }
    }

    /*
    // 计算 Otsu 阈值
    uint8 threshold = otsuThreshold((uint8*)mt9v03x_image);
    printf("Otsu 阈值: %d\n", threshold);

    // 二值化处理
    binarizeImages(mt9v03x_image, mt9v03x_imagebin, threshold);

    // 执行车道线中线识别
    find_lane_midline();
    */


   // processImage(mt9v03x_image, mt9v03x_imagebin);

    find_lane_midline0();
	compress_grayscale_image(mt9v03x_image, mt9v03x_image_compressed, compress_ratio);
    detect_jump_features2();
     leftcricle();
	 rightcricle();
	 outcricle();
     start();
     right_rectan();
     left_rectan();
     get_carmid();

    /*-----------------------------打印leap_flag值-----------------------------*/
    printf("\n打印leap_flag值\n");
    printf("leap_flag = %d", leap_flag);
    printf("\n");
    printf("\n");
    

    /*-----------------------------打印leftcricle_flag值-----------------------------*/
    printf("\n打印leftcricle_flag值\n");
    printf("leftcricle_flag = %d", leftcricle_flag);
    printf("\n");
    printf("\n");
    
    /*-----------------------------打印leftcricle_flag值-----------------------------*/
    printf("\n打印rightcricle_flag值\n");
    printf("rightcricle_flag = %d", rightcricle_flag);
    printf("\n");
    printf("\n");

    /*-----------------------------打印outtcricle_flag值-----------------------------*/
    printf("\n打印outtcricle_flag值\n");
    printf("outtcricle_flag = %d", outtcricle_flag);
    printf("\n");
    printf("\n");


    /*-----------------------------打印startdiffer_flag值-----------------------------*/
    printf("\n打印startdiffer_flag值\n");
    printf("startdiffer_flag = %d", startdiffer_flag);
    printf("\n");
    printf("\n");





    /*-----------------------------打印左/右/中/中边界数组-----------------------------*/
    // 打印表头
    printf("Row    Left    Right    mid_line    carmid_line\n");
    printf("--------------------------------------------------\n");

    // 遍历每一行，打印五列数据
    for (int row = 0; row < MT9V03X_H; row++) {
        printf("%-5d\t%-5d\t%-5d\t%-5d\t%-5u\n",
            row,                   // 行号
            left_line[row],        // 左边键
            right_line[row],       // 右边键
            mid_line[row],         // 中线
            carmid_line[row]);     // 车中线
    }

    printf("\n");


    /*-----------------------------打印部分灰度图-----------------------------*/
    printf("\n打印前几行原始灰度图像数据\n");
    for (int i = 0; i < 5; i++) {
        for (int j = 120; j < 140; j++) {
            printf("%3d ", mt9v03x_image[i][j]);
        }
        printf("\n");
    }


    /*-----------------------------打印部分二值图----------------------------*/
    printf("\n打印前几行二值化图像数据\n");
    for (int i = 0; i < 5; i++) {
        for (int j = 120; j < 140; j++) {
            printf("%3d ", mt9v03x_imagebin[i][j]);
        }
        printf("\n");
    }









    /*----------------------------------------------保存二值图像-------------------------------------------------------*/
    {
        uint8* binary_data = (uint8*)malloc(MT9V03X_H * MT9V03X_W);
        if (!binary_data) {
            printf("内存分配失败，无法保存二值化图像。\n");
        }
        else {
            // 转换二维数组为一维
            for (int i = 0; i < MT9V03X_H; i++) {
                for (int j = 0; j < MT9V03X_W; j++) {
                    binary_data[i * MT9V03X_W + j] = mt9v03x_imagebin[i][j];
                }
            }

            stbi_write_png("binary_output.png", MT9V03X_W, MT9V03X_H, 1, binary_data, MT9V03X_W);
            printf("二值化图像已保存为 binary_output.png\n");

            free(binary_data);
        }
    }

    /*----------------------------------------------保存跳跃点图像-------------------------------------------------------*/
    {
        // 分配内存用于存储图像数据（每个像素1字节）
        uint8_t* leap_data = (uint8_t*)malloc(MT9V03X_H/2 * MT9V03X_W/2);
        if (!leap_data) {
            printf("内存分配失败，无法保存跳跃点图像。\n");
        }
        else {
            // 将二维跳跃点数组转换为一维，并转换为黑白值（1→255白色，0→0黑色）
            for (int y = 0; y < MT9V03X_H/2; y++) {
                for (int x = 0; x < MT9V03X_W/2; x++) {
                    // 注意：leap[x][y] 是二维数组，需要转换为一维索引
                    int index = y * MT9V03X_W/2 + x;
                    // 转换：1 → 白色(255)，0 → 黑色(0)
                    leap_data[index] = (leap[x][y] == 1) ? 255 : 0;
                }
            }

            // 使用 STB 图像库保存为 PNG 文件
            stbi_write_png("leap_points.png", MT9V03X_W/2, MT9V03X_H/2, 1, leap_data, MT9V03X_W/2);
            printf("跳跃点图像已保存为 leap_points.png\n");

            // 释放内存
            free(leap_data);
        }
    }


    /*----------------------------------------------保存压缩后的图像-------------------------------------------------------*/
    {
        // 计算压缩后的图像尺寸
        int compressed_h = MT9V03X_H / compress_ratio;
        int compressed_w = MT9V03X_W / compress_ratio;

        // 动态分配一维数组用于保存图像数据
        uint8_t* compressed_data = (uint8_t*)malloc(compressed_h * compressed_w);
        if (!compressed_data) {
            printf("内存分配失败，无法保存压缩图像。\n");
        }
        else {
            // 将二维数组转换为一维数组
            for (int i = 0; i < compressed_h; i++) {
                for (int j = 0; j < compressed_w; j++) {
                    compressed_data[i * compressed_w + j] = mt9v03x_image_compressed[i][j];
                }
            }

            // 使用 stbi_write_png 保存图像
            stbi_write_png("compressed_output.png", compressed_w, compressed_h, 1, compressed_data, compressed_w);

            printf("压缩图像已保存为 compressed_output.png\n");

            // 释放内存
            free(compressed_data);
        }
    }
    

    /*----------------------------------------------保存原图带边界的图像-------------------------------------------------------*/
    {
        uint8* output_img = (uint8*)malloc(MT9V03X_H * MT9V03X_W * CHANNELS);
        if (!output_img) {
            printf("内存分配失败，无法生成带边界的图像。\n");
        }
        else {
            // 将原始灰度图转换为 RGB 图像
            for (int i = 0; i < MT9V03X_H; i++) {
                for (int j = 0; j < MT9V03X_W; j++) {
                    int idx = i * MT9V03X_W + j;
                    output_img[idx * CHANNELS + 0] = mt9v03x_image[i][j]; // R
                    output_img[idx * CHANNELS + 1] = mt9v03x_image[i][j]; // G
                    output_img[idx * CHANNELS + 2] = mt9v03x_image[i][j]; // B
                }
            }

            // 绘制左边界（绿色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = left_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    output_img[idx * CHANNELS + 0] = 0;   // R
                    output_img[idx * CHANNELS + 1] = 255; // G
                    output_img[idx * CHANNELS + 2] = 0;   // B
                }
            }

            // 绘制中线（红色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = mid_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    output_img[idx * CHANNELS + 0] = 255; // R
                    output_img[idx * CHANNELS + 1] = 0;   // G
                    output_img[idx * CHANNELS + 2] = 0;   // B
                }
            }

            // 绘制右边界（蓝色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = right_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    output_img[idx * CHANNELS + 0] = 0;   // R
                    output_img[idx * CHANNELS + 1] = 0;   // G
                    output_img[idx * CHANNELS + 2] = 255; // B
                }
            }

            // 绘制眼睛（黄色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = carmid_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    output_img[idx * CHANNELS + 0] = 255;   // R
                    output_img[idx * CHANNELS + 1] = 255;   // G
                    output_img[idx * CHANNELS + 2] = 0; // B
                }
            }

            // 保存图像
            stbi_write_png("output_with_boundaries.png", MT9V03X_W, MT9V03X_H, CHANNELS, output_img, MT9V03X_W * CHANNELS);
            printf("带绿线（左边界）、红线（中线）、蓝线（右边界）黄色（眼睛）的图像已保存为 output_with_boundaries.png\n");

            // 释放内存
            free(output_img);
        }
    }

    // 释放内存
    stbi_image_free(img);


    /*----------------------------------------------保存二值图像带边界的图像-------------------------------------------------------*/
    {
        // 为二值图像的 RGB 版本分配内存
        uint8* binary_output_img = (uint8*)malloc(MT9V03X_H * MT9V03X_W * CHANNELS);
        if (!binary_output_img) {
            printf("内存分配失败，无法生成带边界的二值图像。\n");
        }
        else {
            // 将二值图像转换为 RGB 图像
            for (int i = 0; i < MT9V03X_H; i++) {
                for (int j = 0; j < MT9V03X_W; j++) {
                    int idx = i * MT9V03X_W + j;
                    uint8 pixel = mt9v03x_imagebin[i][j];
                    binary_output_img[idx * CHANNELS + 0] = pixel; // R
                    binary_output_img[idx * CHANNELS + 1] = pixel; // G
                    binary_output_img[idx * CHANNELS + 2] = pixel; // B
                }
            }

            // 绘制左边界（绿色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = left_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    binary_output_img[idx * CHANNELS + 0] = 0;   // R
                    binary_output_img[idx * CHANNELS + 1] = 255; // G
                    binary_output_img[idx * CHANNELS + 2] = 0;   // B
                }
            }

            // 绘制中线（红色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = mid_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    binary_output_img[idx * CHANNELS + 0] = 255; // R
                    binary_output_img[idx * CHANNELS + 1] = 0;   // G
                    binary_output_img[idx * CHANNELS + 2] = 0;   // B
                }
            }

            // 绘制右边界（蓝色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = right_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    binary_output_img[idx * CHANNELS + 0] = 0;   // R
                    binary_output_img[idx * CHANNELS + 1] = 0;   // G
                    binary_output_img[idx * CHANNELS + 2] = 255; // B
                }
            }

            // 绘制眼睛（黄色）
            for (int row = 0; row < MT9V03X_H; row++) {
                int col = carmid_line[row];
                if (col >= 0 && col < MT9V03X_W) {
                    int idx = row * MT9V03X_W + col;
                    binary_output_img[idx * CHANNELS + 0] = 255;   // R
                    binary_output_img[idx * CHANNELS + 1] = 255;   // G
                    binary_output_img[idx * CHANNELS + 2] = 0; // B
                }
            }

            // 保存二值图像（带边界线）
            stbi_write_png("output_binary_with_boundaries.png", MT9V03X_W, MT9V03X_H, CHANNELS, binary_output_img, MT9V03X_W * CHANNELS);
            printf("带绿线（左边界）、红线（中线）、蓝线（右边界）黄色（眼睛）的二值图像已保存为 output_binary_with_boundaries.png\n");

            // 释放内存
            free(binary_output_img);
        }
    }

    return 0;
}