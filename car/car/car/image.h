#ifndef __IMAGE_H
#define __IMAGE_H

// 类型定义
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;



// 替换 stdint.h（如果系统不支持）
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif





/*----------------------------------------------宏定义-------------------------------------------------------*/

#define MT9V03X_H     120      // 图像高度
#define MT9V03X_W     188      // 图像宽度
#define SEARCH_RANGE  90      // 搜索偏移量 ±15 列






/*----------------------------------------------全局变量-------------------------------------------------------*/

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];           // 原始灰度图
extern uint8 mt9v03x_imagebin[MT9V03X_H][MT9V03X_W];        // 二值化图像
extern uint8 mid_line[MT9V03X_H];                           // 存储每一行的中线位置（视觉顺序）
extern uint8 left_line[MT9V03X_H];                           // 存储每一行的左边界位置（视觉顺序）
extern uint8 right_line[MT9V03X_H];                           // 存储每一行的右边界位置（视觉顺序）

extern uint8 carmid_line[MT9V03X_H];       //小车真正的中线

extern uint8 lossline_threshold_rectan ;       //直角上方无线的阈值
extern uint8 lossline_threshold_rectanless;       //直角上方无线相差的阈值


/*----------------------------------------------函数声明-------------------------------------------------------*/
void binarizeImages(uint8 image[MT9V03X_H][MT9V03X_W], uint8 imagebin[MT9V03X_H][MT9V03X_W], int threshold);
uint8 otsuThreshold(uint8* image);
void find_lane_midline(void);

#endif