// PID.h 保持不变，仅展示关键注释说明
#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Target;     // 目标值
    float Actual;     // 实际值
    float Out;        // 输出值

    float Kp;         // 线性P增益
    float Ki;         // 非线性P增益（原Ki字段重用为Kp2）
    float Kd;         // D增益

    float Error0;     // 当前误差
    float Error1;     // 上一次误差
    float ErrorInt;   // 积分误差（本控制不使用）

    float Outmax;     // 输出上限
    float Outmin;     // 输出下限

}PID_t;

// 注意：使用时需将Kp2参数赋值给Ki字段
void PID_Update(PID_t* p);

#endif