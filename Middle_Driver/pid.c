#include "pid.h"
#include <math.h>
// 
// @简介：PID初始化函数
// 
void PID_Init(PID_InitTypeDef *PID_InitStruct)
{
    PID_InitStruct->Kp = STD_KP;
    PID_InitStruct->Ki = STD_KI;
    PID_InitStruct->Kd = STD_KD;
    PID_InitStruct->Target_Speed = 0.0f;
    PID_InitStruct->Current_Speed = 0.0f;
    PID_InitStruct->Current_Error = 0.0f;
    PID_InitStruct->Last_Error = 0.0f;
    PID_InitStruct->integral = 0.0f;
    PID_InitStruct->Integral_Max = INTEGRAL_MAX_LIMIT;
    PID_InitStruct->Integral_Min = INTEGRAL_MIN_LIMIT;
    PID_InitStruct->PWM_Max = PWM_MAX_LIMIT;
    PID_InitStruct->PWM_Min = PWM_MIN_LIMIT;
    PID_InitStruct->Dead_Band = DEAD_BAND_WIDTH;
}

//
// @简介：PID调节函数
//
void PID_SetPID(PID_InitTypeDef *PID_InitStruct, float New_Kp, float New_Ki, float New_Kd)
{
    PID_InitStruct->Kp = New_Kp;
    PID_InitStruct->Ki = New_Ki;
    PID_InitStruct->Kd = New_Kd;
}

//
// @简介：设置目标速度,同时消除目标突变带来的微分冲击
//
void PID_SetTarget_Speed(PID_InitTypeDef *PID_InitStruct, float New_Target_Speed)
{
    PID_InitStruct->Target_Speed = New_Target_Speed;

    PID_InitStruct->Current_Error = PID_InitStruct->Target_Speed - PID_InitStruct->Current_Speed;
    PID_InitStruct->Last_Error = PID_InitStruct->Current_Error;
}

//
// @简介：从编码器获取当前速度
//
void PID_UpdateCurrent_Speed(PID_InitTypeDef *PID_InitStruct, float Current_Speed)
{
    PID_InitStruct->Current_Speed = Current_Speed;
}

//
// @简介：计算输出PWM（CCR）并返回
//
int32_t PID_Calc(PID_InitTypeDef *PID_InitStruct)
{
    PID_InitStruct->Current_Error = PID_InitStruct->Target_Speed - PID_InitStruct->Current_Speed;

    float PWM = 0;

    if(fabsf(PID_InitStruct->Current_Error) < PID_InitStruct->Dead_Band)
    {
        PID_InitStruct->Last_Error = PID_InitStruct->Current_Error;
        PWM = PID_InitStruct->Ki * PID_InitStruct->integral;
    }
    else
    {
        PID_InitStruct->integral += PID_InitStruct->Current_Error;

        float p = PID_InitStruct->Kp * PID_InitStruct->Current_Error;

        if(PID_InitStruct->integral > PID_InitStruct->Integral_Max)
        {
            PID_InitStruct->integral = PID_InitStruct->Integral_Max;
        }
        else if(PID_InitStruct->integral < PID_InitStruct->Integral_Min)
        {
            PID_InitStruct->integral = PID_InitStruct->Integral_Min;
        }
        float i = PID_InitStruct->Ki * PID_InitStruct->integral;

        float d = PID_InitStruct->Kd * (PID_InitStruct->Current_Error - PID_InitStruct->Last_Error);

        PWM = p + i + d;

        PID_InitStruct->Last_Error = PID_InitStruct->Current_Error;
    }

    if(PWM > (float)PID_InitStruct->PWM_Max) 
    {
        PWM = (float)PID_InitStruct->PWM_Max;
    }
    else if(PWM < (float)PID_InitStruct->PWM_Min)
    {
        PWM = (float)PID_InitStruct->PWM_Min;
    }

    if(PWM >= 0.0f) 
    {
        return (int32_t)(PWM + 0.5f);
    }
    else
    {
        return (int32_t)(PWM - 0.5f);
    }
}
