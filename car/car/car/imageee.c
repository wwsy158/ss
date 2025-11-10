
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "image.h"
#include "math.h"








/*----------------------------------------------全局变量-------------------------------------------------------*/

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];           // 原始灰度图
uint8 mt9v03x_imagebin[MT9V03X_H][MT9V03X_W];        // 二值化图像


// 压缩后的图像缓冲区（假设压缩比例为 2）
uint8 mt9v03x_image_compressed[(MT9V03X_H) / 2][(MT9V03X_W) / 2];  //压缩图像

// 压缩比例（必须 >= 1）
uint8 compress_ratio = 2;// 压缩比例


uint8 left_line[MT9V03X_H];                           // 存储每一行的左边界位置（视觉顺序）
uint8 right_line[MT9V03X_H];                           // 存储每一行的右边界位置（视觉顺序）
uint8 carmid_line[MT9V03X_H];       //小车真正的中线
uint8 mid_line[MT9V03X_H];                           // 存储每一行的中线位置（视觉顺序）






uint8 lossline_threshold_rectan = 30;       //直角上方无线的阈值
uint8 lossline_threshold_rectanless = 5;       //直角上方无线相差的阈值

uint8 left_count = 0;               // 存储丢失左边界的行数
uint8 right_count = 0;              // 存储丢失右边界的行数

uint8 leap_flag = 0;//是否有跳跃点

uint8 leftcricle_flag = 0;//标志位

uint8 rightcricle_flag = 0;//标志位

uint8 outtcricle_flag = 0;//标志位

// 用于保存跳跃点的二维数组（1表示存在跳跃点，0表示无）
uint8 leap[MT9V03X_W/2][MT9V03X_H/2];

uint8 leap_point_count = 0;

uint8 start_times = 0;
uint8 startdiffer_flag = 0;//标志位


uint8 ll_rectan = 0;
uint8 rr_rectan = 0;

uint8 dash_flag = 0;//虚线轨道判断标志位









/*----------------------------------------------函数实现-------------------------------------------------------*/

void thresholdBinary(uint8 image[MT9V03X_H][MT9V03X_W],
    uint8 out[MT9V03X_H][MT9V03X_W],
    uint8 thresh,
    uint8 maxValue)
{
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < MT9V03X_W; j++) {
            out[i][j] = (image[i][j] > thresh) ? maxValue : 0;
        }
    }
}

/**
 * @brief 使用 Laplacian 算子进行二阶边缘检测
 * @param imageIn 输入图像
 * @param imageOut 输出边缘图像
 * @param Threshold 边缘检测阈值
 */
void laplacian(uint8 imageIn[MT9V03X_H][MT9V03X_W],
    uint8 imageOut[MT9V03X_H][MT9V03X_W],
    uint8 Threshold)
{
    const int kernel_size = 3;
    const int half_kernel = kernel_size / 2;

    // Laplacian 核（可选不同核）
    const int8_t kernel[3][3] = {
        {0,  1, 0},
        {1, -4, 1},
        {0,  1, 0}
    };

    for (int i = half_kernel; i < MT9V03X_H - half_kernel; i++) {
        for (int j = half_kernel; j < MT9V03X_W - half_kernel; j++) {
            int sum = 0;
            for (int ki = -half_kernel; ki <= half_kernel; ki++) {
                for (int kj = -half_kernel; kj <= half_kernel; kj++) {
                    sum += imageIn[i + ki][j + kj] * kernel[ki + 1][kj + 1];
                }
            }
            // 取绝对值并限制范围
            imageOut[i][j] = (abs(sum) > Threshold) ? 255 : 0;
        }
    }

    // 边界补零
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < half_kernel; j++) {
            imageOut[i][j] = 0;
            imageOut[i][MT9V03X_W - 1 - j] = 0;
        }
    }
    for (int j = 0; j < MT9V03X_W; j++) {
        for (int i = 0; i < half_kernel; i++) {
            imageOut[i][j] = 0;
            imageOut[MT9V03X_H - 1 - i][j] = 0;
        }
    }
}


/**
 * @brief 优化的Otsu动态阈值计算
 * @param image 输入灰度图像指针
 * @param col 图像列宽
 * @param row 图像行高
 * @return 计算得到的动态阈值
 */
uint8 otsuThreshold1(uint8* image, uint16_t col, uint16_t row)
{
#define GRAY_SCALE 256
    uint32_t* hist = (uint32_t*)calloc(GRAY_SCALE, sizeof(uint32_t));
    if (!hist) {
        return 128; // 默认阈值
    }

    uint16_t width = col;
    uint16_t height = row;
    uint32_t totalPixels = 0;
    uint32_t sumPixels = 0;
    uint8 threshold = 0;

    for (uint16_t y = height / 10; y < height * 9 / 10; y++) {
        for (uint16_t x = width / 10; x < width * 9 / 10; x++) {
            uint8 pixel = image[y * width + x];
            hist[pixel]++;
            sumPixels += pixel;
            totalPixels++;
        }
    }

    if (totalPixels == 0) {
        free(hist);
        return 128;
    }

    uint8 minVal = 0, maxVal = GRAY_SCALE - 1;
    while (minVal < GRAY_SCALE && hist[minVal] == 0) minVal++;
    while (maxVal > minVal && hist[maxVal] == 0) maxVal--;
    if (minVal == maxVal || minVal + 1 == maxVal) {
        free(hist);
        return minVal;
    }

    double omegaBack = 0, muBack = 0, muTotal = (double)sumPixels / totalPixels;
    double maxSigma = -1.0;
    uint32_t pixelBack = 0;
    uint32_t sumBack = 0;

    for (uint8 t = minVal; t <= maxVal; t++) {
        pixelBack += hist[t];
        sumBack += hist[t] * t;
        omegaBack = (double)pixelBack / totalPixels;
        double omegaFore = 1.0 - omegaBack;
        if (omegaFore < 1e-6) break;
        muBack = (double)sumBack / pixelBack;
        double muFore = (muTotal - omegaBack * muBack) / omegaFore;
        double sigma = omegaBack * omegaFore * (muBack - muFore) * (muBack - muFore);
        if (sigma > maxSigma) {
            maxSigma = sigma;
            threshold = (uint8)t;
        }
    }

    free(hist);
    return threshold;
}


void gaussianBlur(uint8 imageIn[MT9V03X_H][MT9V03X_W], uint8 imageOut[MT9V03X_H][MT9V03X_W])
{
    const int kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };
    const int sum = 16;

    for (int i = 1; i < MT9V03X_H - 1; i++) {
        for (int j = 1; j < MT9V03X_W - 1; j++) {
            int val = 0;
            for (int ki = -1; ki <= 1; ki++) {
                for (int kj = -1; kj <= 1; kj++) {
                    val += imageIn[i + ki][j + kj] * kernel[ki + 1][kj + 1];
                }
            }
            imageOut[i][j] = val / sum;
        }
    }
}

/*
 * @brief 改进的Sobel边缘检测
 * @param imageIn 输入图像
 * @param imageOut 输出边缘图像
 * @param Threshold 动态阈值
 */
void sobel(uint8 imageIn[MT9V03X_H][MT9V03X_W],
    uint8 imageOut[MT9V03X_H][MT9V03X_W],
    uint8 Threshold)
{


    const short KERNEL_SIZE = 3;
    const short xStart = KERNEL_SIZE / 2;
    const short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
    const short yStart = KERNEL_SIZE / 2;
    const short yEnd = MT9V03X_H - KERNEL_SIZE / 2;

    // Sobel核系数
    const int8_t sobel_x[3][3] = {
        {-1,  0,  1},
        {-3,  0,  3},  // 中间行权重加大
        {-1,  0,  1}
    };

    const int8_t sobel_y[3][3] = {
        {-1, -3, -1},
        { 0,  0,  0},
        { 1,  3,  1}
    };

    // 边缘检测主循环
    for (short i = yStart; i < yEnd; i++) {
        for (short j = xStart; j < xEnd; j++) {
            int32_t grad_x = 0, grad_y = 0;

            // 应用Sobel核
            for (short ki = -1; ki <= 1; ki++) {
                for (short kj = -1; kj <= 1; kj++) {
                    int pixel = imageIn[i + ki][j + kj];
                    grad_x += pixel * sobel_x[ki + 1][kj + 1];
                    grad_y += pixel * sobel_y[ki + 1][kj + 1];
                }
            }

            // 计算梯度幅值
            int32_t magnitude = (int32_t)sqrt(grad_x * grad_x + grad_y * grad_y);

            // 阈值处理
            imageOut[i][j] = (magnitude > Threshold) ? 255 : 0;
        }
    }
}

/**
 * @brief 主处理函数：先动态阈值分割，后边缘检测
 */
void processImage(uint8 src[MT9V03X_H][MT9V03X_W],
    uint8 dst[MT9V03X_H][MT9V03X_W])
{
    // 1. 计算动态阈值
    uint8 threshold = otsuThreshold1((uint8*)src, MT9V03X_W, MT9V03X_H);

    // 2. 应用阈值进行预分割（可选）
    uint8 temp[MT9V03X_H][MT9V03X_W];
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < MT9V03X_W; j++) {
            temp[i][j] = (src[i][j] > threshold) ? 255 : src[i][j];
        }
    }

    // 3. 使用 Laplacian 进行边缘检测
    laplacian(temp, dst, threshold * 1.2);  // 使用类似机制设置阈值
}












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

void compress_grayscale_image(
    uint8 src[MT9V03X_H][MT9V03X_W],
    uint8 dst[MT9V03X_H / 2][MT9V03X_W / 2],  // 示例：压缩比例为2
    uint8 ratio
) {
    if (ratio == 0) {
        // 防止除零错误
        return;
    }

    // 压缩后的图像尺寸
    int new_h = (MT9V03X_H + ratio - 1) / ratio;  // 向上取整
    int new_w = (MT9V03X_W + ratio - 1) / ratio;

    for (int y = 0; y < new_h; y++) {
        for (int x = 0; x < new_w; x++) {
            // 计算原始图像中对应的区域
            int src_y = y * ratio;
            int src_x = x * ratio;

            // 保证不越界
            if (src_y >= MT9V03X_H) src_y = MT9V03X_H - 1;
            if (src_x >= MT9V03X_W) src_x = MT9V03X_W - 1;

            // 取块的左上角像素作为压缩后的像素
            dst[y][x] = src[src_y][src_x];
        }
    }
}

// 计算两个像素值之间的差比和
float get_difference_ratio(uint8 a, uint8 b) {
    float diff = abs(a - b);
    float sum = a + b;
    return sum == 0 ? 0 : diff / sum;
}


void detect_jump_features1(void)
{
    // 差比和参数（改为相邻点）
#define THRESHOLD_RATIO 0.2f      // 相邻点差异较小，阈值应更低     0.2
#define REQUIRED_POINTS 4     //每行需要检测的跳跃点数量
#define REQUIRED_ROWS 5   // 连续多少行满足条件

     leap_flag = 0;//初始化必清零
    int consecutive_rows = 0;

    for (int y = MT9V03X_H / 2 - 1; y >= 0; y--) {
        int jump_count = 0;

        // 扫描当前行，比较相邻像素点
        for (int x = 0; x < MT9V03X_W / 2 - 1; x++) {
            uint8 a = mt9v03x_image_compressed[y][x];
            uint8 b = mt9v03x_image_compressed[y][x + 1];  // 相邻点

            float ratio = get_difference_ratio(a, b);

            if (ratio > THRESHOLD_RATIO) {
                jump_count++;
            }
        }

        // 判断当前行是否满足跳跃点数量要求
        if (jump_count >= REQUIRED_POINTS) {
            consecutive_rows++;
        }
        else {
            consecutive_rows = 0;
        }

        // 连续五行满足条件时，置标志位
        if (consecutive_rows >= REQUIRED_ROWS) {
            leap_flag = 1;

            left_count = 0;  //避免直角道误判
            right_count = 0;

            break;
        }
    }

}







void detect_jump_features2(void)
{
    // 差比和参数
#define THRESHOLD_RATIO 0.2f      // 相邻点差异阈值
#define REQUIRED_POINTS 4         // 每行至少检测到的跳跃点数量
#define REQUIRED_ROWS 5           // 连续多少行满足条件

// 清除标志位和跳跃点数组
    leap_flag = 0;
    leap_point_count = 0;
    memset(leap, 0, sizeof(leap));  // 初始化为 0，表示没有跳跃点

    int consecutive_rows = 0;

    // 从图像中部向上扫描
    for (int y = MT9V03X_H / 2 - 1; y >= 0; y--)
    {
        int jump_count = 0;

        // 扫描当前行，比较相邻像素点
        for (int x = 0; x < MT9V03X_W / 2 - 1; x++)
        {
            uint8 a = mt9v03x_image_compressed[y][x];
            uint8 b = mt9v03x_image_compressed[y][x + 1];  // 相邻点

            float ratio = get_difference_ratio(a, b);

            if (ratio > THRESHOLD_RATIO)
            {
                jump_count++;

                // 存储跳跃点坐标
                if (x < MT9V03X_W && y < MT9V03X_H)
                {
                    leap[x][y] = 1;  // 标记跳跃点
                    leap_point_count++;
                }
            }
        }

        // 判断当前行是否满足跳跃点数量要求
        if (jump_count >= REQUIRED_POINTS)
        {
            consecutive_rows++;
        }
        else
        {
            consecutive_rows = 0;
        }

        // 连续五行满足条件时，置标志位
        if (consecutive_rows >= REQUIRED_ROWS)
        {
            leap_flag = 1;

            left_count = 0;  // 避免直角道误判
            right_count = 0;

            break;
        }
    }
}


















/**
 * @brief 使用差比和方法检测赛道边界并提取中线
 */
void find_lane_midline0(void)
{

    /*

        // 定义搜索范围（左右各 SEARCH_RANGE 像素）
    #define SEARCH_RANGE 80
    #define THRESHOLD_RATIO 0.05f   // 差比和阈值（经验值）

    #define STEP_SIZE 1  // 像素间隔（建议 3~7）

        uint8 prev_left = MAX(0, (MT9V03X_W / 2) - SEARCH_RANGE);
        uint8 prev_right = MIN(MT9V03X_W - 1, (MT9V03X_W / 2) + SEARCH_RANGE);
        uint8 prev_mid = (prev_left + prev_right) / 2;

        uint8 left_count = 0;               // 存储丢失左边界的行数
        uint8 right_count = 0;              // 存储丢失右边界的行数
        uint8 left_flag = 0;                // 左边界标志
        uint8 right_flag = 0;               // 右边界标志

        // 遍历图像的每一行（从下到上）
        for (int sensor_row = MT9V03X_H - 1; sensor_row >= 0; sensor_row--)
        {
            int visual_row = sensor_row;

            uint8 current_left = prev_left;
            uint8 current_right = prev_right;

            left_flag = 0;
            right_flag = 0;

            // 左边界搜索：向右滑动查找跳变点
            for (int col = MAX(1, prev_mid - SEARCH_RANGE);
                col <= MIN(MT9V03X_W - STEP_SIZE - 1, prev_mid + SEARCH_RANGE);
                col++)
            {
                uint8 left_pixel = mt9v03x_image[sensor_row][col];
                uint8 right_pixel = mt9v03x_image[sensor_row][col + STEP_SIZE];

                float diff = left_pixel - right_pixel;
                float sum = left_pixel + right_pixel + 1e-6f; // 防止除零
                float ratio = fabs(diff / sum);

                if (ratio > THRESHOLD_RATIO && left_pixel > right_pixel)
                {
                    current_left = col;
                    prev_left = col;
                    left_count = 0;
                    left_flag = 1;
                    break;
                }
            }


            // 如果未找到左边界，计数器加一
            if (left_flag == 0) {
                left_count++;
            }

            // 右边界搜索：向左滑动查找跳变点
            for (int col = MIN(MT9V03X_W - STEP_SIZE - 1, prev_mid + SEARCH_RANGE);
                col >= MAX(1, prev_mid - SEARCH_RANGE);
                col--)
            {
                uint8 left_pixel = mt9v03x_image[sensor_row][col];
                uint8 right_pixel = mt9v03x_image[sensor_row][col + STEP_SIZE];

                float diff = right_pixel - left_pixel;
                float sum = left_pixel + right_pixel + 1e-6f;
                float ratio = fabs(diff / sum);

                if (ratio > THRESHOLD_RATIO && right_pixel > left_pixel)
                {
                    current_right = col + STEP_SIZE;
                    prev_right = col + STEP_SIZE;
                    right_count = 0;
                    right_flag = 1;
                    break;
                }
            }


            // 如果未找到右边界，计数器加一
            if (right_flag == 0) {
                right_count++;
            }

            // 更新数组
            left_line[visual_row] = current_left;
            right_line[visual_row] = current_right;
            mid_line[visual_row] = (current_left + current_right) / 2;

            // 更新中线参考点
            prev_mid = mid_line[visual_row];
        }

    */




    // 定义搜索范围（左右各 SEARCH_RANGE 像素）
#define SEARCH_RANGE 80
#define THRESHOLD_RATIO 0.05f   // 差比和阈值（经验值）

    uint8 prev_left = MAX(0, (MT9V03X_W / 2) - SEARCH_RANGE);
    uint8 prev_right = MIN(MT9V03X_W - 1, (MT9V03X_W / 2) + SEARCH_RANGE);
    uint8 prev_mid = (prev_left + prev_right) / 2;

    left_count = 0;               // 存储丢失左边界的行数 初始化给0
    right_count = 0;              // 存储丢失右边界的行数 0
    uint8 left_flag = 0;                // 左边界标志
    uint8 right_flag = 0;               // 右边界标志

    // 遍历图像的每一行（从下到上）
    for (int sensor_row = MT9V03X_H - 1; sensor_row >= 0; sensor_row--)
    {
        int visual_row = sensor_row;

        uint8 current_left = prev_left;
        uint8 current_right = prev_right;

        left_flag = 0;
        right_flag = 0;

        // 左边界搜索：向右滑动查找跳变点
        for (int col = MAX(1, prev_mid - SEARCH_RANGE);
            col <= MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col++)
        {
            uint8 left_pixel = mt9v03x_image[sensor_row][col - 1];
            uint8 curr_pixel = mt9v03x_image[sensor_row][col];
            uint8 right_pixel = mt9v03x_image[sensor_row][col + 1];

            // 计算差比和
            float diff = left_pixel - curr_pixel;
            float sum = left_pixel + curr_pixel + 1e-6f; // 防止除零
            float ratio = diff / sum;

            if (fabs(ratio) > THRESHOLD_RATIO && curr_pixel > left_pixel)
            {
                current_left = col;
                prev_left = col;
                left_count = 0;
                left_flag = 1;
                break;
            }
        }

        // 如果未找到左边界，计数器加一
        if (left_flag == 0) {
            left_count++;
        }

        // 右边界搜索：向左滑动查找跳变点
        for (int col = MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col >= MAX(1, prev_mid - SEARCH_RANGE);
            col--)
        {
            uint8 left_pixel = mt9v03x_image[sensor_row][col - 1];
            uint8 curr_pixel = mt9v03x_image[sensor_row][col];
            uint8 right_pixel = mt9v03x_image[sensor_row][col + 1];

            // 计算差比和
            float diff = curr_pixel - right_pixel;
            float sum = curr_pixel + right_pixel + 1e-6f; // 防止除零
            float ratio = diff / sum;

            if (fabs(ratio) > THRESHOLD_RATIO && curr_pixel > right_pixel)
            {
                current_right = col;
                prev_right = col;
                right_count = 0;
                right_flag = 1;
                break;
            }
        }

        // 如果未找到右边界，计数器加一
        if (right_flag == 0) {
            right_count++;
        }

        // 更新数组
        left_line[visual_row] = current_left;
        right_line[visual_row] = current_right;
        mid_line[visual_row] = (current_left + current_right) / 2;

        // 更新中线参考点
        prev_mid = mid_line[visual_row];
    }

    /*-----------------------------打印right_count和left_count值-----------------------------*/
    printf("\n打印right_count和left_count值\n");
    printf("right_count = %d, left_count = %d\n", right_count, left_count);
    printf("\n");

}


void get_carmid(void)
{
    // 逐个元素复制
    for (int i = 0; i < MT9V03X_H; i++) {
        carmid_line[i] = mid_line[i];
    }
	if (rr_rectan == 1)
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
	if (ll_rectan == 1)
	{
		// 前 left_count 个元素赋值为 0
		for (int i = 0; i < left_count; i++) {
			carmid_line[i] = 0;
		}
		// 剩余元素复制自 mid_line
		for (int i = left_count; i < MT9V03X_H; i++) {
			carmid_line[i] = mid_line[i];
		}
	}
    if (leftcricle_flag == 1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = left_line[i];
        }
    }
	if (rightcricle_flag == 1)
	{
		// 逐个元素复制
		for (int i = 0; i < MT9V03X_H; i++) {
			carmid_line[i] = right_line[i];
		}
	}

    if (outtcricle_flag == 1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = left_line[i];//后面再改
        }
    }


}










void dash(void)
{

     dash_flag = 0;//虚线轨道判断标志位
    uint8 dash_differ = (right_count > left_count) ? (right_count - left_count) : (left_count - right_count);
    if (right_count > 70 && left_count > 70 && dash_differ < 3)
    {
        dash_flag = 1;

    }

    /*-----------------------------打印dash_flag值-----------------------------*/
    printf("\n打印dash_flag值\n");
    printf("dash_flag = %d", dash_flag);
    printf("\n");
    printf("\n");
}




void right_rectan(void)
{
    /*----------------------------------------------右直角轨道-------------------------------------------------------*/
     rr_rectan = 0;
    if (((right_count > lossline_threshold_rectan) || (left_count > lossline_threshold_rectan))
        && ((right_count - left_count) > lossline_threshold_rectanless))
    {
        rr_rectan = 1;
    }

    /*-----------------------------打印rr_rectan值-----------------------------*/
    printf("\n打印rr_rectan值\n");
    printf("rr_rectan = %d", rr_rectan);
    printf("\n");
    printf("\n");

}





void left_rectan(void)
{
    /*----------------------------------------------左直角轨道-------------------------------------------------------*/
   ll_rectan = 0;
    if (((right_count > lossline_threshold_rectan) || (left_count > lossline_threshold_rectan))
        && ((left_count - right_count) > lossline_threshold_rectanless))
    {

        ll_rectan = 1;
    }

    /*-----------------------------打印ll_rectan值-----------------------------*/
    printf("\n打印ll_rectan值\n");
    printf("ll_rectan = %d", ll_rectan);
    printf("\n");
    printf("\n");
}




void start(void)
{
	uint8 startdiffer_count = 0;//计数值
	 startdiffer_flag = 0;//标志位
    start_times = 0;//标志位
    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        if (right_line[i] - left_line[i] > 70 && (abs((right_line[i] - left_line[i]) - (right_line[i + 4] - left_line[i + 4])) < 2)) {
            startdiffer_count++;
        }
        else {
            startdiffer_count = 0;
        }
            if (startdiffer_count >= 6&&i>90) {
                startdiffer_flag = 1;
                break;
            }
        }
    }

/*
if (startdiffer_flag) {
	start_times++;
	if (start_times >= 2) {
		start_flag = 1;
	}
}
else {
	start_times = 0;
}
}
*/








//有点悬,线写着
void outcricle(void)
{
    outtcricle_flag = 0;
    if (!leftcricle_flag && !rightcricle_flag&& leap_flag)
    {
        outtcricle_flag = 1;
    }

}



void leftcricle(void)
{

    uint8 leftcricle_lucount = 0;//左环岛左边界上面几行的点计数值
    uint8 leftcricle_ldcount = 0;//满足的点

    uint8 leftcricle_rucount = 0;//满足的点
    uint8 leftcricle_rdcount = 0;//满足的点

    uint8 leftcricle_luflag = 0;//左环岛左边界上面几行的标志位
    uint8 leftcricle_ldflag = 0;

    uint8 leftcricle_ruflag = 0;
    uint8 leftcricle_rdflag = 0;

	uint8 leftcricle_preflag = 0;//预测标志位
     leftcricle_flag = 0;//标志位

    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        leftcricle_lucount = 0;
        leftcricle_ldcount = 0;

        leftcricle_rucount = 0;
        leftcricle_rdcount = 0;

        leftcricle_luflag = 0; 
        leftcricle_ldflag = 0;

        leftcricle_ruflag = 0;
        leftcricle_rdflag = 0;


        // 检查左侧线的连续6行条件(下面6行）是否小于某行  left 
        for (int j = i; j < MT9V03X_H; j++) {

            if (left_line[i] - left_line[j] > 4) {
                leftcricle_ldcount++;
                if (leftcricle_ldcount >= 6) {
                    leftcricle_ldflag = 1;
                    break;
                }

            }
        }



        // 检查左侧线的连续6行条件(上面6行）是否小于某行 left 
        for (int j = i; j > 0; j--) {

            if (left_line[i] - left_line[j] > 4) {
                leftcricle_lucount++;
                if (leftcricle_lucount >= 6) {
                    leftcricle_luflag = 1;
                    break;
                }

            }
        }

        if (leftcricle_ldflag && leftcricle_luflag)
        {

            // 检查右侧线的连续6行条件(下面6行）是否接近某行  left 
            for (int j = i; j < MT9V03X_H; j++) {

                if (abs(right_line[i] - right_line[j]) < 1) {
                    leftcricle_rdcount++;
                    if (leftcricle_rdcount >= 6) {
                        leftcricle_rdflag = 1;
                        break;
                    }

                }
            }


            // 检右侧线的连续6行条件(上面6行）是否接近某行 left 
            for (int j = i; j > 0; j--) {

                if (abs(right_line[i] - right_line[j]) < 1) {
                    leftcricle_rucount++;
                    if (leftcricle_rucount >= 6) {
                        leftcricle_ruflag = 1;
                        break;
                    }

                }
            }


        }

        //及时退出,已经检测到左环岛了
        if (leftcricle_rdflag && leftcricle_ruflag) {
            leftcricle_preflag = 1;
            break;
        }

    }
	if (leftcricle_preflag && leap_flag)
	{
		leftcricle_flag = 1;
	}


}



void rightcricle(void)
{

    uint8 rightcricle_lucount = 0;//左环岛左边界上面几行的点计数值
    uint8 rightcricle_ldcount = 0;//满足的点
          
    uint8 rightcricle_rucount = 0;//满足的点
    uint8 rightcricle_rdcount = 0;//满足的点
         
    uint8 rightcricle_luflag = 0;//左环岛左边界上面几行的标志位
    uint8 rightcricle_ldflag = 0;
         
    uint8 rightcricle_ruflag = 0;
    uint8 rightcricle_rdflag = 0;
          

    uint8 rightcricle_preflag = 0;//预测标志位
     rightcricle_flag = 0;//真正标志位

    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        rightcricle_lucount = 0;
        rightcricle_ldcount = 0;

        rightcricle_rucount = 0;
        rightcricle_rdcount = 0;

        rightcricle_luflag = 0; 
        rightcricle_ldflag = 0;

        rightcricle_ruflag = 0;
        rightcricle_rdflag = 0;


        // 检查左侧线的连续6行条件(下面6行）是否小于某行  right 
        for (int j = i; j < MT9V03X_H; j++) {

            if (right_line[j] - right_line[i] > 4) {
                rightcricle_ldcount++;
                if (rightcricle_ldcount >= 6) {
                    rightcricle_ldflag = 1;
                    break;
                }

            }
        }



        // 检查左侧线的连续6行条件(上面6行）是否小于某行 right 
        for (int j = i; j > 0; j--) {

            if (right_line[j] - right_line[i] > 4) {
                rightcricle_lucount++;
                if (rightcricle_lucount >= 6) {
                    rightcricle_luflag = 1;
                    break;
                }

            }
        }

        if (rightcricle_ldflag && rightcricle_luflag)
        {

            // 检查右侧线的连续6行条件(下面6行）是否接近某行  right
            for (int j = i; j < MT9V03X_H; j++) {

                if (abs(left_line[i] - left_line[j]) < 1) {
                    rightcricle_rdcount++;
                    if (rightcricle_rdcount >= 6) {
                        rightcricle_rdflag = 1;
                        break;
                    }

                }
            }


            // 检右侧线的连续6行条件(上面6行）是否接近某行 right 
            for (int j = i; j > 0; j--) {

                if (abs(left_line[i] - left_line[j]) < 1) {
                    rightcricle_rucount++;
                    if (rightcricle_rucount >= 6) {
                        rightcricle_ruflag = 1;
                        break;
                    }

                }
            }


        }

        //及时退出,已经检测到左环岛了
        if (rightcricle_rdflag && rightcricle_ruflag) {
            rightcricle_preflag = 1;
            break;
        }




    }



	if (rightcricle_preflag&&leap_flag)
	{
		rightcricle_flag = 1;
	}
}























void aa(void)
{
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
            if (left_line[j] - left_line[i] > 6) {
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

        for (int j = i; j < MT9V03X_H - 1; j++) {

            if (right_line[j] - left_line[j] > 70 && (abs((right_line[j] - left_line[j]) - (right_line[j + 1] - left_line[j + 1])) == 0)) {
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
    if (startdiffer_flag == 1 && lstart_flag == 1 && rstart_flag == 1 && (lstart > 90))
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
            if (left_line[i] - left_line[j] > 5) {
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
            leftcircle_flag = 1;break;
        }

    }




    /*----------------------------------------------右环岛预处理-------------------------------------------------------*/
    for (int i = 4; i <= MT9V03X_H - 5; i++) {



        // 检查左边是否有至少4个点大于当前点
        for (int j = 0; j < i; j++) {
            if (right_line[j] - right_line[i] > 5) {
                rleft_count++;
                if (rleft_count >= 4) {
                    rleft4 = 1;
                    break;
                }

            }
        }
        // 检查右边是否有至少4个点大于当前点
        for (int j = i + 1; j < MT9V03X_H; j++) {
            if (right_line[j] - right_line[i] > 5) {
                rright_count++;
                if (rright_count >= 4) {
                    rright4 = 1;
                    break;
                }
            }
        }
        if (rleft4 && rright4 == 1) {
            rightcircle_flag = 1;break;
        }

    }







    /*----------------------------------------------出环岛预处理-------------------------------------------------------*/
    uint8 outcricle_lcount = 0;//满足的点
    uint8 outcricle_rcount = 0;//满足的点

    uint8 outcricle_lflag = 0;
    uint8 outcricle_rflag = 0;

    uint8 outcricle_flag = 0;//出环岛标志位


    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        // 检查右侧线的连续6行条件
        for (int j = i; j < MT9V03X_H; j++) {
            if (abs(right_line[j] - right_line[i]) < 2) {
                outcricle_rcount++;
                if (outcricle_rcount >= 6) {
                    outcricle_rflag = 1;
                    break;
                }

            }
        }
        // 检查左边是否有至少4个点大于当前点
        for (int j = i; j < MIN(i + 10, MT9V03X_H); j++) {
            if (left_line[j] - left_line[i] > 0 && left_line[j] != left_line[j + 1]) {
                outcricle_lcount++;
                if (outcricle_lcount >= 6) {
                    outcricle_lflag = 1;
                    break;
                }
            }
        }
        if (outcricle_rflag && outcricle_lflag == 1) {
            outcricle_flag = 1;break;
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

    /*-----------------------------打印outcricle_flag值-----------------------------*/
    printf("\n打印outcricle_flag值\n");
    printf("outcricle_flag = %d", outcricle_flag);
    printf("\n");
    printf("\n");



    /*----------------------------------------------直轨道，十字路口，折线-------------------------------------------------------*/
    uint8 yes = 0;
    if (right_count < 2 && left_count < 2)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = mid_line[i];
            yes = 1;
        }
    }

    /*-----------------------------打印yes值-----------------------------*/
    printf("\n打印yes值\n");
    printf("yes = %d", yes);
    printf("\n");
    printf("\n");



    /*----------------------------------------------右直角轨道-------------------------------------------------------*/
    uint8 rr_rectan = 0;
    if (((right_count > lossline_threshold_rectan) || (left_count > lossline_threshold_rectan))
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
        rr_rectan = 1;

    }

    /*-----------------------------打印rr_rectan值-----------------------------*/
    printf("\n打印rr_rectan值\n");
    printf("rr_rectan = %d", rr_rectan);
    printf("\n");
    printf("\n");



    /*----------------------------------------------左直角轨道-------------------------------------------------------*/
    uint8 ll_rectan = 0;
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
        ll_rectan = 1;
    }

    /*-----------------------------打印ll_rectan值-----------------------------*/
    printf("\n打印ll_rectan值\n");
    printf("ll_rectan = %d", ll_rectan);
    printf("\n");
    printf("\n");


    /*----------------------------------------------左环岛轨道-------------------------------------------------------*/
    uint8 llee_flag = (leftcircle_flag == 1 && rightcircle_flag == 0);
    if (llee_flag == 1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = left_line[i];
        }

    }
    /*-----------------------------打印llee_flag值-----------------------------*/
    printf("\n打印llee_flag值\n");
    printf("llee_flag = %d", llee_flag);
    printf("\n");
    printf("\n");



    /*----------------------------------------------右环岛轨道-------------------------------------------------------*/
    uint8 rrii_flag = (rightcircle_flag == 1 && leftcircle_flag == 0);
    if (rrii_flag == 1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = right_line[i];
        }

    }

    /*-----------------------------打印rrii_flag值-----------------------------*/
    printf("\n打印rrii_flag值\n");
    printf("rrii_flag = %d", rrii_flag);
    printf("\n");
    printf("\n");







    /*----------------------------------------------虚线轨道判断-------------------------------------------------------*/
    uint8 dash_flag = 0;//虚线轨道判断标志位
    uint8 dash_differ = (right_count > left_count) ? (right_count - left_count) : (left_count - right_count);
    if (right_count > 70 && left_count > 70 && dash_differ < 3)
    {
        dash_flag = 1;

    }

    /*-----------------------------打印dash_flag值-----------------------------*/
    printf("\n打印dash_flag值\n");
    printf("dash_flag = %d", dash_flag);
    printf("\n");
    printf("\n");




    //-----------------------------转角值的处理并求图片的差值-----------------------------//
    uint32 image_differ = 0;
    int image_differsum = 0;
    uint32 image_changshu = 93 * 120;
    for (int i = 0; i < MT9V03X_H; i++) {
        image_differ += carmid_line[i];
    }
    image_differsum = image_differ - image_changshu;

    //-----------------------------打印图片的差值-----------------------------//
    printf("\n打印image_differsum值\n");
    printf("image_differsum = %d", image_differsum);
    printf("\n");
    printf("\n");



}






























































/**
 * @brief 使用差比和方法检测赛道边界并提取中线
 */
void find_lane_midline1(void)
{
    // 定义搜索范围（左右各 SEARCH_RANGE 像素）
#define SEARCH_RANGE 80
#define THRESHOLD_RATIO 0.06f   // 差比和阈值（经验值）

    uint8 prev_left = MAX(0, (MT9V03X_W / 2) - SEARCH_RANGE);
    uint8 prev_right = MIN(MT9V03X_W - 1, (MT9V03X_W / 2) + SEARCH_RANGE);
    uint8 prev_mid = (prev_left + prev_right) / 2;

    uint8 left_count = 0;               // 存储丢失左边界的行数
    uint8 right_count = 0;              // 存储丢失右边界的行数
    uint8 left_flag = 0;                // 左边界标志
    uint8 right_flag = 0;               // 右边界标志

    // 遍历图像的每一行（从下到上）
    for (int sensor_row = MT9V03X_H - 1; sensor_row >= 0; sensor_row--)
    {
        int visual_row = sensor_row;

        uint8 current_left = prev_left;
        uint8 current_right = prev_right;

        left_flag = 0;
        right_flag = 0;

        // 左边界搜索：向右滑动查找跳变点
        for (int col = MAX(1, prev_mid - SEARCH_RANGE);
            col <= MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col++)
        {
            uint8 left_pixel = mt9v03x_image[sensor_row][col - 1];
            uint8 curr_pixel = mt9v03x_image[sensor_row][col];
            uint8 right_pixel = mt9v03x_image[sensor_row][col + 1];

            // 计算差比和
            float diff = left_pixel - curr_pixel;
            float sum = left_pixel + curr_pixel + 1e-6f; // 防止除零
            float ratio = diff / sum;

            if (fabs(ratio) > THRESHOLD_RATIO && curr_pixel > left_pixel)
            {
                current_left = col;
                prev_left = col;
                left_count = 0;
                left_flag = 1;
                break;
            }
        }

        // 如果未找到左边界，计数器加一
        if (left_flag == 0) {
            left_count++;
        }

        // 右边界搜索：向左滑动查找跳变点
        for (int col = MIN(MT9V03X_W - 2, prev_mid + SEARCH_RANGE);
            col >= MAX(1, prev_mid - SEARCH_RANGE);
            col--)
        {
            uint8 left_pixel = mt9v03x_image[sensor_row][col - 1];
            uint8 curr_pixel = mt9v03x_image[sensor_row][col];
            uint8 right_pixel = mt9v03x_image[sensor_row][col + 1];

            // 计算差比和
            float diff = curr_pixel - right_pixel;
            float sum = curr_pixel + right_pixel + 1e-6f; // 防止除零
            float ratio = diff / sum;

            if (fabs(ratio) > THRESHOLD_RATIO && curr_pixel > right_pixel)
            {
                current_right = col;
                prev_right = col;
                right_count = 0;
                right_flag = 1;
                break;
            }
        }

        // 如果未找到右边界，计数器加一
        if (right_flag == 0) {
            right_count++;
        }

        // 更新数组
        left_line[visual_row] = current_left;
        right_line[visual_row] = current_right;
        mid_line[visual_row] = (current_left + current_right) / 2;

        // 更新中线参考点
        prev_mid = mid_line[visual_row];
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
            if (left_line[j] - left_line[i] > 6) {
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

        for (int j = i; j < MT9V03X_H - 1; j++) {

            if (right_line[j] - left_line[j] > 70 && (abs((right_line[j] - left_line[j]) - (right_line[j + 1] - left_line[j + 1])) == 0)) {
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
    if (startdiffer_flag == 1 && lstart_flag == 1 && rstart_flag == 1 && (lstart > 90))
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
            if (left_line[i] - left_line[j] > 5) {
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
            leftcircle_flag = 1;break;
        }

    }




    /*----------------------------------------------右环岛预处理-------------------------------------------------------*/
    for (int i = 4; i <= MT9V03X_H - 5; i++) {



        // 检查左边是否有至少4个点大于当前点
        for (int j = 0; j < i; j++) {
            if (right_line[j] - right_line[i] > 5) {
                rleft_count++;
                if (rleft_count >= 4) {
                    rleft4 = 1;
                    break;
                }

            }
        }
        // 检查右边是否有至少4个点大于当前点
        for (int j = i + 1; j < MT9V03X_H; j++) {
            if (right_line[j] - right_line[i] > 5) {
                rright_count++;
                if (rright_count >= 4) {
                    rright4 = 1;
                    break;
                }
            }
        }
        if (rleft4 && rright4 == 1) {
            rightcircle_flag = 1;break;
        }

    }







    /*----------------------------------------------出环岛预处理-------------------------------------------------------*/
    uint8 outcricle_lcount = 0;//满足的点
    uint8 outcricle_rcount = 0;//满足的点

    uint8 outcricle_lflag = 0;
    uint8 outcricle_rflag = 0;

    uint8 outcricle_flag = 0;//出环岛标志位


    for (int i = MT9V03X_H - 1; i >= 6; i--) {

        // 检查右侧线的连续6行条件
        for (int j = i; j < MT9V03X_H; j++) {
            if (abs(right_line[j] - right_line[i]) < 2) {
                outcricle_rcount++;
                if (outcricle_rcount >= 6) {
                    outcricle_rflag = 1;
                    break;
                }

            }
        }
        // 检查左边是否有至少4个点大于当前点
        for (int j = i; j < MIN(i + 10, MT9V03X_H); j++) {
            if (left_line[j] - left_line[i] > 0 && left_line[j] != left_line[j + 1]) {
                outcricle_lcount++;
                if (outcricle_lcount >= 6) {
                    outcricle_lflag = 1;
                    break;
                }
            }
        }
        if (outcricle_rflag && outcricle_lflag == 1) {
            outcricle_flag = 1;break;
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

    /*-----------------------------打印outcricle_flag值-----------------------------*/
    printf("\n打印outcricle_flag值\n");
    printf("outcricle_flag = %d", outcricle_flag);
    printf("\n");
    printf("\n");



    /*----------------------------------------------直轨道，十字路口，折线-------------------------------------------------------*/
    uint8 yes=0;
    if (right_count < 2 && left_count < 2)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = mid_line[i];
            yes = 1;
        }
    }

    /*-----------------------------打印yes值-----------------------------*/
    printf("\n打印yes值\n");
    printf("yes = %d", yes);
    printf("\n");
    printf("\n");



    /*----------------------------------------------右直角轨道-------------------------------------------------------*/
    uint8 rr_rectan = 0;
    if (((right_count > lossline_threshold_rectan) || (left_count > lossline_threshold_rectan))
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
        rr_rectan = 1;

    }

    /*-----------------------------打印rr_rectan值-----------------------------*/
    printf("\n打印rr_rectan值\n");
    printf("rr_rectan = %d", rr_rectan);
    printf("\n");
    printf("\n");



    /*----------------------------------------------左直角轨道-------------------------------------------------------*/
    uint8 ll_rectan = 0;
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
        ll_rectan = 1;
    }

    /*-----------------------------打印ll_rectan值-----------------------------*/
    printf("\n打印ll_rectan值\n");
    printf("ll_rectan = %d", ll_rectan);
    printf("\n");
    printf("\n");


    /*----------------------------------------------左环岛轨道-------------------------------------------------------*/
    uint8 llee_flag=(leftcircle_flag == 1 && rightcircle_flag == 0);
    if (llee_flag==1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = left_line[i];
        }

    }
    /*-----------------------------打印llee_flag值-----------------------------*/
    printf("\n打印llee_flag值\n");
    printf("llee_flag = %d", llee_flag);
    printf("\n");
    printf("\n");



    /*----------------------------------------------右环岛轨道-------------------------------------------------------*/
    uint8 rrii_flag = (rightcircle_flag == 1 && leftcircle_flag == 0);
    if (rrii_flag==1)
    {
        // 逐个元素复制
        for (int i = 0; i < MT9V03X_H; i++) {
            carmid_line[i] = right_line[i];
        }

    }

    /*-----------------------------打印rrii_flag值-----------------------------*/
    printf("\n打印rrii_flag值\n");
    printf("rrii_flag = %d", rrii_flag);
    printf("\n");
    printf("\n");







    /*----------------------------------------------虚线轨道判断-------------------------------------------------------*/
    uint8 dash_flag = 0;//虚线轨道判断标志位
    uint8 dash_differ = (right_count > left_count) ? (right_count - left_count) : (left_count - right_count);
    if (right_count > 70 && left_count > 70 && dash_differ < 3)
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




// 赛道中线识别
void find_lane_midline2(void) {


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

























