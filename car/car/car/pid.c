#include "PID.h"
#include <math.h>  // 用于绝对值函数

void PID_Update(PID_t* p)
{
    // 更新误差值
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    // 双PD控制公式：
    // 转角值 = Kp*Error0 + Kp2*Error0*|Error0| + Kd*(Error0 - Error1)

    p->Out = p->Kp * p->Error0 +               // 线性P项
             p->Ki * p->Error0 * fabs(p->Error0) +  // 非线性P项（原Ki作为Kp2使用）
             p->Kd * (p->Error0 - p->Error1);   // D项

    // 输出限幅
    if (p->Out > p->Outmax)
    {
        p->Out = p->Outmax;
    }
    if (p->Out < p->Outmin)
    {
        p->Out = p->Outmin;
    }
}